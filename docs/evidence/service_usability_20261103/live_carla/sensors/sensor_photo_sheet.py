#!/usr/bin/env python3
"""HH_261103 - Label unchanged actual JPEGs outside their pixels; never invent frames, objects or continuous motion."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[5]
ORDER = ("baseline_end", "front_10cm", "front_20cm", "front_30cm", "front_40cm", "roadside_left", "roadside_right")


def load_photos(manifest):
    rows = [row for row in map(json.loads, manifest.read_text().splitlines()) if row.get("image")]
    if len(rows) != 7 or {row["case"] for row in rows} != set(ORDER):
        raise ValueError("expected exactly seven actual case photographs")
    photos = []
    for case in ORDER:
        row = next(row for row in rows if row["case"] == case)
        path = Path(row["image"])
        path = path if path.is_absolute() else ROOT/path
        if hashlib.sha256(path.read_bytes()).hexdigest() != row["image_sha256"]:
            raise ValueError("source JPEG SHA mismatch: "+case)
        receipt = row["receipt"]
        label = "No owned test actor" if case == "baseline_end" else f'BBox front {receipt["measured_front_body_gap_m"]*100:.4f} cm'
        if case.startswith("roadside_"):
            body, actor = receipt["ego_bounds_m"]["right"], receipt["obstacle_bounds_m"]["right"]
            label += f'; side {max(body[0]-actor[1], actor[0]-body[1])*100:.4f} cm'
        photos.append({"case": case, "label": label, "capture_utc": datetime.fromisoformat(row["utc"]).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]+" UTC",
            "path": path, "source_sha256": row["image_sha256"], "source_stamp_s": row["source_stamp_s"]})
    return photos


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output-prefix", type=Path, required=True)
    args = parser.parse_args()
    outputs = {kind: args.output_prefix.with_name(args.output_prefix.name+suffix)
        for kind, suffix in (("png", ".png"), ("gif", ".gif"), ("manifest", "_manifest.json"))}
    if any(path.exists() for path in outputs.values()):
        parser.error("choose a new prefix; source and final evidence are never overwritten")
    photos = load_photos(args.manifest)
    font = lambda size: ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", size)
    sheet = Image.new("RGB", (1280, 1390), "white")
    draw = ImageDraw.Draw(sheet)
    draw.text((12, 12), "ACTUAL CARLA CAMERA — SEPARATE STATIONARY CASES", font=font(28), fill="black")
    draw.text((12, 52), "Screenshot sequence, NOT continuous video. NOT a braking test.", font=font(23), fill="#9a2200")
    frames = []
    for index, photo in enumerate(photos):
        with Image.open(photo["path"]) as original:
            original = original.convert("RGB")
        if original.size != (800, 600):
            raise ValueError("unexpected actual camera dimensions")
        x, y = (index % 3)*426+2, 106+(index//3)*410
        draw.text((x+4, y), photo["case"], font=font(23), fill="black")
        draw.text((x+4, y+30), photo["label"], font=font(16), fill="black")
        draw.text((x+4, y+53), photo["capture_utc"], font=font(16), fill="black")
        # HH_261103 - Nearest-neighbor thumbnail sampling adds no interpolated scene pixels; originals remain untouched.
        sheet.paste(original.resize((424, 318), Image.Resampling.NEAREST), (x, y+80))
        frame = Image.new("RGB", (960, 726), "white")
        caption = ImageDraw.Draw(frame)
        caption.text((12, 8), "ACTUAL CAMERA SEQUENCE — NOT CONTINUOUS / NOT BRAKING", font=font(23), fill="#9a2200")
        caption.text((12, 40), photo["case"]+" | "+photo["label"], font=font(21), fill="black")
        caption.text((12, 72), "Captured "+photo["capture_utc"], font=font(21), fill="black")
        frame.paste(original, (80, 118))  # Native 800×600 pixels; no GIF image/frame interpolation.
        frames.append(frame.convert("P", palette=Image.Palette.ADAPTIVE, colors=256, dither=Image.Dither.NONE))
    draw.text((440, 1050), "Original JPEGs retained with SHA-256.", font=font(20), fill="black")
    draw.text((440, 1090), "BBox gap is NOT nearest mesh skin.", font=font(20), fill="black")
    draw.text((440, 1130), "Roadside subjects may be outside front-camera FOV.", font=font(19), fill="black")
    draw.text((440, 1170), "No object drawing, AI imagery or frame interpolation.", font=font(19), fill="black")
    draw.text((12, 1350), "2026-11-03 actual03 | Seven independent photographs; timestamps above are capture UTC.", font=font(20), fill="black")
    with outputs["png"].open("xb") as target:
        sheet.save(target, format="PNG", optimize=True)
    with outputs["gif"].open("xb") as target:
        frames[0].save(target, format="GIF", save_all=True, append_images=frames[1:], duration=1500, loop=0, disposal=2, optimize=False)
    artifacts = {kind: {"path": str(path), "bytes": path.stat().st_size, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        for kind, path in outputs.items() if kind != "manifest"}
    assert all(item["bytes"] < 5_000_000 for item in artifacts.values()), "visual exceeds requested size cap"
    record = {"evidence_type": "ACTUAL_CAMERA_SCREENSHOT_SEQUENCE_NOT_CONTINUOUS_NOT_BRAKING",
        "source_manifest": str(args.manifest), "source_manifest_sha256": hashlib.sha256(args.manifest.read_bytes()).hexdigest(),
        "generator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), "artifacts": artifacts,
        "rendering": "PNG nearest-neighbor thumbnails only; GIF native 800x600 JPEG decode with palette quantization. Labels outside scene pixels. No object drawing, AI or frame interpolation.",
        "gif_frames": 7, "gif_frame_duration_ms": 1500, "png_dimensions": list(sheet.size), "gif_dimensions": [960, 726],
        "photos": [{**photo, "path": str(photo["path"])} for photo in photos]}
    with outputs["manifest"].open("x") as target:
        json.dump(record, target, ensure_ascii=False, indent=2)
        target.write("\n")
    print(json.dumps(artifacts))


if __name__ == "__main__":
    main()
