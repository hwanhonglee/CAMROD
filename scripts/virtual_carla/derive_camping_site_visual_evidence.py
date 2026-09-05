#!/usr/bin/env python3
"""Derive per-site PNG/GIF evidence from one completed matrix capture.

This is an offline, non-controlling postprocessor.  It maps the UTC timestamps
in a completed ``camping_site_matrix.json`` onto an already-recorded constant-
frame-rate X11 MP4.  For every attempted site it emits a six-frame contact
sheet and a short GIF made only from frames inside that site's measured
``started_at_utc``/``finished_at_utc`` interval.

The output directory must not exist.  Input/schema/time/frame validation is
fail closed and all output is staged before an atomic rename.  The program
does not start CARLA/ROS, send UI or vehicle commands, synthesize frames, or
use AI generation/enhancement.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Mapping, Sequence


SCHEMA = "camrod.virtual_carla.camping_site_visual_evidence.v1"
MATRIX_SCHEMA = "camrod.virtual_carla.camping_site_matrix.v1"
FINAL_REPORT_STATUSES = {"PASS", "FAIL"}
FINAL_SITE_STATUSES = {"PASS", "FAIL"}
SITE_RE = re.compile(r"^B([1-9]|1[0-3])$")
SAMPLE_COUNT = 6


class EvidenceError(RuntimeError):
    """A fail-closed source, timing, media, or derivation error."""


@dataclass(frozen=True)
class VideoInfo:
    duration_s: float
    fps: float
    frame_count: int
    width_px: int
    height_px: int
    codec_name: str
    start_time_s: float


@dataclass(frozen=True)
class TimedEvent:
    at_utc: datetime
    event: str


@dataclass(frozen=True)
class SiteWindow:
    site: str
    status: str
    started_at_utc: datetime
    finished_at_utc: datetime
    start_offset_s: float
    finish_offset_s: float
    start_frame: int
    finish_frame: int
    milestones: tuple[TimedEvent, ...]


@dataclass(frozen=True)
class RepresentativeSample:
    frame: int
    video_offset_s: float
    at_utc: str
    source: str
    events: tuple[str, ...]


@dataclass(frozen=True)
class GifInterval:
    start_s: float
    end_s: float


def _duplicate_key_rejector(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise EvidenceError(f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise EvidenceError(f"{name} must be a JSON object")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as error:
        raise EvidenceError(f"cannot hash {path}: {error}") from None
    return digest.hexdigest()


def _artifact(path: Path, root: Path) -> dict[str, Any]:
    if not path.is_file() or path.stat().st_size <= 0:
        raise EvidenceError(f"derived artifact is missing or empty: {path}")
    return {
        "path": path.relative_to(root).as_posix(),
        "bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def parse_utc(value: Any, name: str) -> datetime:
    """Parse an explicitly UTC ISO-8601 value; naive/non-UTC input rejects."""
    if not isinstance(value, str) or not value.strip():
        raise EvidenceError(f"{name} must be a non-empty UTC timestamp")
    raw = value.strip()
    normalized = raw[:-1] + "+00:00" if raw.endswith("Z") else raw
    try:
        parsed = datetime.fromisoformat(normalized)
    except ValueError:
        raise EvidenceError(f"{name} is not a valid ISO-8601 timestamp: {raw!r}") from None
    if parsed.tzinfo is None or parsed.utcoffset() != timedelta(0):
        raise EvidenceError(f"{name} must use UTC (Z or +00:00): {raw!r}")
    return parsed.astimezone(timezone.utc)


def format_utc(value: datetime) -> str:
    return value.astimezone(timezone.utc).isoformat(timespec="milliseconds").replace(
        "+00:00", "Z"
    )


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise EvidenceError(f"{name} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise EvidenceError(f"{name} must be a finite number: {value!r}") from None
    if not math.isfinite(number):
        raise EvidenceError(f"{name} must be a finite number: {value!r}")
    return number


def _ratio(value: Any, name: str) -> float:
    if not isinstance(value, str) or "/" not in value:
        raise EvidenceError(f"{name} must be an ffprobe rational: {value!r}")
    numerator_raw, denominator_raw = value.split("/", 1)
    numerator = _finite(numerator_raw, f"{name} numerator")
    denominator = _finite(denominator_raw, f"{name} denominator")
    if numerator <= 0.0 or denominator <= 0.0:
        raise EvidenceError(f"{name} must be positive: {value!r}")
    return numerator / denominator


def video_info_from_probe(document: Mapping[str, Any]) -> VideoInfo:
    """Validate the ffprobe contract required for frame-exact selection."""
    streams = document.get("streams")
    if not isinstance(streams, list) or len(streams) != 1:
        raise EvidenceError("source video must expose exactly one selected video stream")
    stream = _mapping(streams[0], "ffprobe.streams[0]")
    fmt = _mapping(document.get("format"), "ffprobe.format")
    duration = _finite(fmt.get("duration"), "ffprobe.format.duration")
    start_time = _finite(stream.get("start_time", 0.0), "ffprobe.stream.start_time")
    fps = _ratio(stream.get("avg_frame_rate"), "ffprobe.stream.avg_frame_rate")
    nominal_fps = _ratio(stream.get("r_frame_rate"), "ffprobe.stream.r_frame_rate")
    if abs(fps - nominal_fps) > max(1e-6, fps * 1e-6):
        raise EvidenceError(
            "source video is not accepted as constant-frame-rate: "
            f"avg={fps:g} nominal={nominal_fps:g}"
        )
    if abs(start_time) > 1.0 / fps:
        raise EvidenceError(
            f"source video stream must start at zero; got {start_time:g}s"
        )
    try:
        frame_count = int(stream.get("nb_read_frames"))
        width = int(stream.get("width"))
        height = int(stream.get("height"))
    except (TypeError, ValueError):
        raise EvidenceError(
            "ffprobe stream is missing integer frame count/width/height"
        ) from None
    if duration <= 0.0 or duration > 86401.0:
        raise EvidenceError(f"source video duration is out of range: {duration:g}s")
    if frame_count < SAMPLE_COUNT or width <= 0 or height <= 0:
        raise EvidenceError(
            "source video has insufficient frames or invalid dimensions: "
            f"frames={frame_count} size={width}x{height}"
        )
    expected_frames = duration * fps
    if abs(expected_frames - frame_count) > 2.5:
        raise EvidenceError(
            "source video duration/frame-rate/frame-count disagree: "
            f"duration*fps={expected_frames:.3f} frames={frame_count}"
        )
    codec = str(stream.get("codec_name", "")).strip()
    if not codec:
        raise EvidenceError("ffprobe stream codec_name is missing")
    return VideoInfo(
        duration_s=duration,
        fps=fps,
        frame_count=frame_count,
        width_px=width,
        height_px=height,
        codec_name=codec,
        start_time_s=start_time,
    )


def _run(command: Sequence[str], commands: list[str]) -> subprocess.CompletedProcess[str]:
    commands.append(shlex.join(str(value) for value in command))
    try:
        completed = subprocess.run(
            [str(value) for value in command],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except OSError as error:
        raise EvidenceError(f"cannot execute {command[0]}: {error}") from None
    if completed.returncode != 0:
        detail = completed.stderr.strip()[-2000:]
        raise EvidenceError(
            f"command failed with exit {completed.returncode}: "
            f"{shlex.join(command)}\n{detail}"
        )
    return completed


def probe_video(path: Path, commands: list[str]) -> tuple[VideoInfo, Mapping[str, Any]]:
    command = [
        "ffprobe", "-v", "error", "-select_streams", "v:0", "-count_frames",
        "-show_entries",
        "stream=codec_name,width,height,start_time,avg_frame_rate,r_frame_rate,nb_read_frames:format=duration",
        "-of", "json", str(path),
    ]
    completed = _run(command, commands)
    try:
        decoded = json.loads(completed.stdout, object_pairs_hook=_duplicate_key_rejector)
    except (json.JSONDecodeError, EvidenceError) as error:
        raise EvidenceError(f"cannot decode ffprobe output: {error}") from None
    probe = _mapping(decoded, "ffprobe output")
    return video_info_from_probe(probe), probe


def read_matrix(path: Path) -> tuple[Mapping[str, Any], str]:
    if not path.is_file():
        raise EvidenceError(f"matrix report is not a regular file: {path}")
    try:
        payload = path.read_bytes()
        decoded = json.loads(
            payload.decode("utf-8"), object_pairs_hook=_duplicate_key_rejector
        )
    except EvidenceError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise EvidenceError(f"cannot decode matrix report {path}: {error}") from None
    report = _mapping(decoded, str(path))
    if report.get("schema") != MATRIX_SCHEMA:
        raise EvidenceError(
            f"matrix schema must be {MATRIX_SCHEMA!r}, got {report.get('schema')!r}"
        )
    status = str(report.get("status", "")).strip().upper()
    if status not in FINAL_REPORT_STATUSES:
        raise EvidenceError(
            f"matrix report must be completed PASS or FAIL, got {status!r}"
        )
    sites = report.get("sites")
    if not isinstance(sites, list) or not sites:
        raise EvidenceError("matrix report sites must be a non-empty list")
    return report, hashlib.sha256(payload).hexdigest()


def build_site_windows(
    report: Mapping[str, Any], capture_start: datetime, video: VideoInfo
) -> tuple[SiteWindow, ...]:
    """Validate final site intervals and map them to source video frames."""
    sites = report.get("sites")
    assert isinstance(sites, list)  # validated by read_matrix or test fixture
    windows: list[SiteWindow] = []
    seen: set[str] = set()
    for index, raw_site in enumerate(sites):
        site = _mapping(raw_site, f"sites[{index}]")
        name = str(site.get("site", "")).strip().upper()
        if not SITE_RE.fullmatch(name):
            raise EvidenceError(f"sites[{index}].site is not B1..B13: {name!r}")
        if name in seen:
            raise EvidenceError(f"matrix report contains duplicate site {name}")
        seen.add(name)
        status = str(site.get("status", "")).strip().upper()
        if status == "NOT_ATTEMPTED":
            continue
        if status not in FINAL_SITE_STATUSES:
            raise EvidenceError(
                f"{name}.status must be PASS, FAIL, or NOT_ATTEMPTED; got {status!r}"
            )
        started = parse_utc(site.get("started_at_utc"), f"{name}.started_at_utc")
        finished = parse_utc(site.get("finished_at_utc"), f"{name}.finished_at_utc")
        if finished <= started:
            raise EvidenceError(f"{name} finished_at_utc must be after started_at_utc")
        start_offset = (started - capture_start).total_seconds()
        finish_offset = (finished - capture_start).total_seconds()
        if start_offset < -1e-6 or finish_offset > video.duration_s + 1e-6:
            raise EvidenceError(
                f"{name} UTC interval [{start_offset:.3f}, {finish_offset:.3f}]s "
                f"is outside source video [0, {video.duration_s:.3f}]s"
            )
        start_offset = max(0.0, start_offset)
        finish_offset = min(video.duration_s, finish_offset)
        start_frame = max(0, int(math.ceil(start_offset * video.fps - 1e-9)))
        finish_frame = min(
            video.frame_count - 1,
            int(math.floor(finish_offset * video.fps + 1e-9)),
        )
        if finish_frame - start_frame + 1 < SAMPLE_COUNT:
            raise EvidenceError(
                f"{name} interval contains fewer than {SAMPLE_COUNT} source frames"
            )
        raw_milestones = site.get("milestones")
        if not isinstance(raw_milestones, list) or not raw_milestones:
            raise EvidenceError(f"{name}.milestones must be a non-empty list")
        milestones: list[TimedEvent] = []
        for milestone_index, raw_milestone in enumerate(raw_milestones):
            milestone = _mapping(
                raw_milestone, f"{name}.milestones[{milestone_index}]"
            )
            at = parse_utc(
                milestone.get("at_utc"),
                f"{name}.milestones[{milestone_index}].at_utc",
            )
            event = str(milestone.get("event", "")).strip()
            if not event:
                raise EvidenceError(
                    f"{name}.milestones[{milestone_index}].event is empty"
                )
            if at < started or at > finished:
                raise EvidenceError(
                    f"{name} milestone {event!r} at {format_utc(at)} is outside "
                    "the site's measured UTC interval"
                )
            milestones.append(TimedEvent(at, event))
        milestones.sort(key=lambda item: (item.at_utc, item.event))
        windows.append(
            SiteWindow(
                site=name,
                status=status,
                started_at_utc=started,
                finished_at_utc=finished,
                start_offset_s=start_offset,
                finish_offset_s=finish_offset,
                start_frame=start_frame,
                finish_frame=finish_frame,
                milestones=tuple(milestones),
            )
        )
    if not windows:
        raise EvidenceError("matrix report has no finalized attempted site")
    report_status = str(report.get("status", "")).strip().upper()
    if report_status == "PASS" and any(window.status != "PASS" for window in windows):
        raise EvidenceError("PASS matrix report contains a non-PASS attempted site")
    if report_status == "FAIL" and not any(window.status == "FAIL" for window in windows):
        raise EvidenceError("FAIL matrix report contains no failed attempted site")
    return tuple(windows)


def _even_frames(start: int, finish: int, count: int) -> list[int]:
    if count == 1:
        return [start]
    return [
        round(start + index * (finish - start) / (count - 1))
        for index in range(count)
    ]


def choose_representative_samples(
    window: SiteWindow,
    capture_start: datetime,
    video: VideoInfo,
    count: int = SAMPLE_COUNT,
) -> tuple[RepresentativeSample, ...]:
    """Choose distinct source frames, preferring measured milestone frames."""
    if count < 2 or window.finish_frame - window.start_frame + 1 < count:
        raise EvidenceError(f"{window.site} cannot provide {count} distinct frames")
    annotations: dict[int, dict[str, Any]] = {
        window.start_frame: {"sources": {"site_boundary"}, "events": {"site started"}},
        window.finish_frame: {"sources": {"site_boundary"}, "events": {"site finished"}},
    }
    for milestone in window.milestones:
        offset = (milestone.at_utc - capture_start).total_seconds()
        frame = int(math.floor(offset * video.fps + 1e-9))
        frame = min(max(frame, window.start_frame), window.finish_frame)
        entry = annotations.setdefault(frame, {"sources": set(), "events": set()})
        entry["sources"].add("milestone")
        entry["events"].add(milestone.event)

    preferred = set(annotations)
    fallback = _even_frames(window.start_frame, window.finish_frame, count)
    preferred.update(fallback)
    ordered = sorted(preferred)
    if len(ordered) > count:
        # Preserve both boundaries and distribute the remaining choices over
        # the actual timeline.  Milestone frames are retained before fallback
        # frames whenever the two compete for the same temporal slot.
        chosen = [ordered[0], ordered[-1]]
        remaining = count - 2
        interior_milestones = [
            frame
            for frame in ordered[1:-1]
            if "milestone" in annotations.get(frame, {}).get("sources", set())
        ]
        if len(interior_milestones) > remaining:
            indices = _even_frames(0, len(interior_milestones) - 1, remaining)
            chosen.extend(interior_milestones[index] for index in indices)
        else:
            chosen.extend(interior_milestones)
            for target in fallback[1:-1]:
                if len(chosen) >= count:
                    break
                candidates = [
                    frame for frame in ordered[1:-1] if frame not in chosen
                ]
                if candidates:
                    chosen.append(min(candidates, key=lambda frame: (abs(frame - target), frame)))
        ordered = sorted(set(chosen))
    if len(ordered) < count:
        for frame in _even_frames(window.start_frame, window.finish_frame, count * 2):
            if frame not in ordered:
                ordered.append(frame)
            if len(ordered) == count:
                break
        ordered.sort()
    if len(ordered) != count or len(set(ordered)) != count:
        raise EvidenceError(
            f"{window.site} representative frame selection is not exactly {count} unique frames"
        )

    samples: list[RepresentativeSample] = []
    for frame in ordered:
        if not window.start_frame <= frame <= window.finish_frame:
            raise EvidenceError(f"{window.site} selected frame escaped the site interval")
        offset = frame / video.fps
        at = capture_start + timedelta(seconds=offset)
        annotation = annotations.get(frame)
        samples.append(
            RepresentativeSample(
                frame=frame,
                video_offset_s=round(offset, 6),
                at_utc=format_utc(at),
                source=(
                    "+".join(sorted(annotation["sources"]))
                    if annotation
                    else "timeline_fallback"
                ),
                events=tuple(sorted(annotation["events"])) if annotation else (),
            )
        )
    if not any("milestone" in sample.source for sample in samples):
        raise EvidenceError(f"{window.site} selection contains no milestone frame")
    return tuple(samples)


def build_gif_intervals(
    samples: Sequence[RepresentativeSample],
    window: SiteWindow,
    video: VideoInfo,
    clip_seconds: float,
) -> tuple[GifInterval, ...]:
    """Center short clips on samples and merge them without leaving the site."""
    clip = _finite(clip_seconds, "clip_seconds")
    if clip < 0.25 or clip > 5.0:
        raise EvidenceError("clip_seconds must be in [0.25, 5.0]")
    intervals: list[list[float]] = []
    for sample in samples:
        start = max(window.start_offset_s, sample.video_offset_s - clip / 2.0)
        end = min(window.finish_offset_s, video.duration_s, start + clip)
        start = max(window.start_offset_s, end - clip)
        if end - start < 1.0 / video.fps - 1e-9:
            raise EvidenceError(f"{window.site} GIF interval is shorter than one frame")
        if intervals and start <= intervals[-1][1] + 0.5 / video.fps:
            intervals[-1][1] = max(intervals[-1][1], end)
        else:
            intervals.append([start, end])
    result = tuple(
        GifInterval(round(start, 6), round(end, 6)) for start, end in intervals
    )
    if not result:
        raise EvidenceError(f"{window.site} has no GIF intervals")
    if any(
        interval.start_s < window.start_offset_s - 1e-6
        or interval.end_s > window.finish_offset_s + 1e-6
        or interval.end_s > video.duration_s + 1e-6
        or interval.end_s <= interval.start_s
        for interval in result
    ):
        raise EvidenceError(f"{window.site} GIF interval escaped validated bounds")
    return result


def _probe_derivative(path: Path, commands: list[str]) -> Mapping[str, Any]:
    command = [
        "ffprobe", "-v", "error", "-select_streams", "v:0", "-count_frames",
        "-show_entries", "stream=codec_name,width,height,nb_read_frames",
        "-of", "json", str(path),
    ]
    completed = _run(command, commands)
    try:
        decoded = json.loads(completed.stdout, object_pairs_hook=_duplicate_key_rejector)
    except (json.JSONDecodeError, EvidenceError) as error:
        raise EvidenceError(f"cannot decode derivative probe for {path}: {error}") from None
    document = _mapping(decoded, f"ffprobe {path}")
    streams = document.get("streams")
    if not isinstance(streams, list) or len(streams) != 1:
        raise EvidenceError(f"derived artifact has no single video stream: {path}")
    stream = _mapping(streams[0], f"ffprobe {path}.streams[0]")
    try:
        width = int(stream.get("width"))
        height = int(stream.get("height"))
        frames = int(stream.get("nb_read_frames"))
    except (TypeError, ValueError):
        raise EvidenceError(f"derived artifact probe is incomplete: {path}") from None
    if width <= 0 or height <= 0 or frames <= 0:
        raise EvidenceError(f"derived artifact probe is invalid: {path}")
    return document


def _derive_site(
    source_video: Path,
    site_root: Path,
    samples: Sequence[RepresentativeSample],
    intervals: Sequence[GifInterval],
    video: VideoInfo,
    derived_width: int,
    gif_fps: int,
    commands: list[str],
) -> tuple[Path, Path, Mapping[str, Any], Mapping[str, Any]]:
    site_root.mkdir(parents=False, exist_ok=False)
    contact = site_root / "representative_contact_sheet.png"
    gif = site_root / "representative_motion.gif"
    frame_expression = "+".join(f"eq(n\\,{sample.frame})" for sample in samples)
    contact_filter = (
        f"select='{frame_expression}',"
        f"scale={derived_width}:-2:flags=lanczos,"
        "tile=2x3:padding=0:margin=0:color=black"
    )
    _run(
        [
            "ffmpeg", "-n", "-hide_banner", "-loglevel", "error",
            "-i", str(source_video), "-vf", contact_filter,
            "-frames:v", "1", "-vsync", "vfr", str(contact),
        ],
        commands,
    )
    interval_expression = "+".join(
        f"between(t\\,{interval.start_s:.6f}\\,{interval.end_s:.6f})"
        for interval in intervals
    )
    gif_filter = (
        f"[0:v]select='{interval_expression}',"
        f"setpts=N/({video.fps:.12g}*TB),"
        f"scale={derived_width}:-2:flags=lanczos,fps={gif_fps},"
        "split[gif_a][gif_b];"
        "[gif_a]palettegen=stats_mode=diff:max_colors=192[palette];"
        "[gif_b][palette]paletteuse=dither=bayer:bayer_scale=3[gif_out]"
    )
    _run(
        [
            "ffmpeg", "-n", "-hide_banner", "-loglevel", "error",
            "-i", str(source_video), "-filter_complex", gif_filter,
            "-map", "[gif_out]", "-loop", "0", str(gif),
        ],
        commands,
    )
    contact_probe = _probe_derivative(contact, commands)
    gif_probe = _probe_derivative(gif, commands)
    return contact, gif, contact_probe, gif_probe


def _write_hashes(root: Path) -> None:
    files = sorted(
        path for path in root.rglob("*")
        if path.is_file() and path.name != "SHA256SUMS"
    )
    lines = [f"{_sha256(path)}  {path.relative_to(root).as_posix()}" for path in files]
    (root / "SHA256SUMS").write_text("\n".join(lines) + "\n", encoding="utf-8")


def derive(args: argparse.Namespace) -> Path:
    source_video = args.video.expanduser().resolve()
    matrix_path = args.matrix_report.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    if not source_video.is_file() or source_video.stat().st_size <= 0:
        raise EvidenceError(f"source video is not a non-empty regular file: {source_video}")
    source_video_bytes = source_video.stat().st_size
    source_video_sha = _sha256(source_video)
    if output.exists():
        raise EvidenceError(f"output directory already exists: {output}")
    if not output.parent.is_dir():
        raise EvidenceError(f"output parent directory does not exist: {output.parent}")
    if not 320 <= args.derived_width <= 1920:
        raise EvidenceError("--derived-width must be in [320, 1920]")
    if not 1 <= args.gif_fps <= 20:
        raise EvidenceError("--gif-fps must be in [1, 20]")

    capture_start = parse_utc(args.capture_start_utc, "--capture-start-utc")
    report, matrix_sha = read_matrix(matrix_path)
    matrix_bytes = matrix_path.stat().st_size
    commands: list[str] = []
    video, source_probe = probe_video(source_video, commands)
    windows = build_site_windows(report, capture_start, video)

    staging = Path(
        tempfile.mkdtemp(prefix=f".{output.name}.staging.", dir=output.parent)
    )
    try:
        site_documents = []
        for window in windows:
            samples = choose_representative_samples(
                window, capture_start, video, SAMPLE_COUNT
            )
            intervals = build_gif_intervals(
                samples, window, video, args.clip_seconds
            )
            site_root = staging / window.site
            contact, gif, contact_probe, gif_probe = _derive_site(
                source_video,
                site_root,
                samples,
                intervals,
                video,
                args.derived_width,
                args.gif_fps,
                commands,
            )
            site_documents.append(
                {
                    "site": window.site,
                    "matrix_status": window.status,
                    "started_at_utc": format_utc(window.started_at_utc),
                    "finished_at_utc": format_utc(window.finished_at_utc),
                    "video_interval_s": {
                        "start": round(window.start_offset_s, 6),
                        "finish": round(window.finish_offset_s, 6),
                    },
                    "source_frame_interval": {
                        "start": window.start_frame,
                        "finish": window.finish_frame,
                    },
                    "representative_samples": [asdict(sample) for sample in samples],
                    "gif_intervals": [asdict(interval) for interval in intervals],
                    "artifacts": {
                        "contact_sheet_png": {
                            **_artifact(contact, staging),
                            "ffprobe": contact_probe,
                        },
                        "representative_motion_gif": {
                            **_artifact(gif, staging),
                            "ffprobe": gif_probe,
                        },
                    },
                }
            )

        # A still-running capture or matrix writer must never be accepted as
        # immutable evidence.  Re-hash both inputs after all ffmpeg reads and
        # fail before publishing the staged directory if either one changed.
        if (
            source_video.stat().st_size != source_video_bytes
            or _sha256(source_video) != source_video_sha
        ):
            raise EvidenceError("source video changed during derivation")
        if matrix_path.stat().st_size != matrix_bytes or _sha256(matrix_path) != matrix_sha:
            raise EvidenceError("matrix report changed during derivation")

        commands_path = staging / "exact_commands.txt"
        commands_path.write_text("\n".join(commands) + "\n", encoding="utf-8")
        manifest = {
            "schema": SCHEMA,
            "status": "PASS",
            "created_at_utc": format_utc(datetime.now(timezone.utc)),
            "scope": {
                "kind": "offline derivatives from a supplied X11 capture MP4",
                "ai_generated_or_enhanced": False,
                "interpolated_or_synthetic_frames": False,
                "vehicle_motion_or_ui_input_sent": False,
                "carla_or_ros_process_started_or_stopped": False,
                "source_video_modified": False,
                "statement": (
                    "Only already-recorded source frames inside each matrix site's "
                    "measured UTC interval were selected. Visual derivatives do not "
                    "by themselves prove mission success or acquisition provenance."
                ),
            },
            "source": {
                "video": {
                    "path": str(source_video),
                    "bytes": source_video_bytes,
                    "sha256": source_video_sha,
                    "capture_start_utc": format_utc(capture_start),
                    "ffprobe": source_probe,
                },
                "camping_site_matrix": {
                    "path": str(matrix_path),
                    "bytes": matrix_path.stat().st_size,
                    "sha256": matrix_sha,
                    "schema": report.get("schema"),
                    "status": report.get("status"),
                },
            },
            "derivation": {
                "sample_count_per_site": SAMPLE_COUNT,
                "derived_width_px": args.derived_width,
                "gif_fps": args.gif_fps,
                "clip_seconds_per_sample": args.clip_seconds,
                "gif_frame_timing": (
                    "ffmpeg fps filter may duplicate or drop source frames for "
                    "playback timing; it performs no motion interpolation"
                ),
                "timestamp_mapping": (
                    "UTC offset from --capture-start-utc, quantized to a validated "
                    "constant-frame-rate source frame"
                ),
                "site_interval_policy": (
                    "all selected frames and GIF intervals are bounded by the site's "
                    "started_at_utc and finished_at_utc"
                ),
            },
            "sites": site_documents,
            "exact_commands": _artifact(commands_path, staging),
        }
        manifest_path = staging / "derivation_manifest.json"
        manifest_path.write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        _write_hashes(staging)
        os.replace(staging, output)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return output


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", required=True, type=Path)
    parser.add_argument("--capture-start-utc", required=True)
    parser.add_argument("--matrix-report", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--derived-width", type=int, default=960)
    parser.add_argument("--gif-fps", type=int, default=8)
    parser.add_argument("--clip-seconds", type=float, default=1.25)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        output = derive(args)
    except EvidenceError as error:
        print(f"[camping-visual-evidence] ERROR: {error}", file=sys.stderr)
        return 2
    print(f"[camping-visual-evidence] PASS: {output}")
    print(f"[camping-visual-evidence] verify: (cd {shlex.quote(str(output))} && sha256sum -c SHA256SUMS)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
