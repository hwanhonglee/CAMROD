"""Pure CARLA radar-to-CAMROD scalar range mapping helpers."""

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


NO_RETURN_EPSILON_M = 0.001


@dataclass(frozen=True)
class RadarChannelSpec:
    """Validated I/O and measurement contract for one radar channel."""

    name: str
    input_topic: str
    output_topic: str
    standard_output_topic: str
    frame_id: str
    min_range_m: float
    max_range_m: float
    field_of_view_rad: float


@dataclass(frozen=True)
class RadarRangeSelection:
    """Nearest valid detection plus accounting used by diagnostics."""

    range_m: float
    input_detection_count: int
    valid_detection_count: int
    has_target: bool


@dataclass(frozen=True)
class CarlaRadarDetection:
    """One detection from the standard CARLA radar PointCloud2 contract."""

    range_m: float
    velocity_mps: float
    azimuth_rad: float
    elevation_rad: float


@dataclass(frozen=True)
class CoMovingNearFieldExclusion:
    """Tight raw-signature exclusion for a proven co-moving CARLA return."""

    channel_name: str
    min_range_m: float
    max_range_m: float
    min_azimuth_rad: float
    max_azimuth_rad: float
    min_elevation_rad: float
    max_elevation_rad: float


def build_co_moving_near_field_exclusions(
    entries: Sequence[str],
    channel_names: Sequence[str],
) -> tuple[CoMovingNearFieldExclusion, ...]:
    """Parse bounded ``channel:r0:r1:a0:a1:e0:e1`` exclusions.

    The hard geometry limits deliberately prevent this CARLA-only mechanism
    from becoming a broad radar disable.  Every exclusion must stay within a
    0.50 m near field and occupy only a small angular cell.
    """
    known_channels = {str(name).strip().lower() for name in channel_names}
    exclusions = []
    for raw_entry in entries:
        parts = [part.strip() for part in str(raw_entry).split(':')]
        if len(parts) != 7:
            raise ValueError(
                'co-moving near-field exclusion must use '
                'CHANNEL:min_range:max_range:min_azimuth:max_azimuth:'
                'min_elevation:max_elevation'
            )
        channel_name = parts[0].lower()
        if channel_name not in known_channels:
            raise ValueError(
                f'unknown radar channel in near-field exclusion: {parts[0]!r}'
            )
        try:
            values = tuple(float(value) for value in parts[1:])
        except ValueError as error:
            raise ValueError(
                f'invalid numeric near-field exclusion: {raw_entry!r}'
            ) from error
        if not all(math.isfinite(value) for value in values):
            raise ValueError('near-field exclusion values must be finite')
        (
            min_range,
            max_range,
            min_azimuth,
            max_azimuth,
            min_elevation,
            max_elevation,
        ) = values
        if not 0.0 <= min_range < max_range <= 0.50:
            raise ValueError(
                'near-field exclusion range must satisfy '
                '0 <= min < max <= 0.50 m'
            )
        if max_range - min_range > 0.15:
            raise ValueError('near-field exclusion range span exceeds 0.15 m')
        if not -math.pi / 2 <= min_azimuth < max_azimuth <= math.pi / 2:
            raise ValueError('near-field exclusion azimuth bounds are invalid')
        if max_azimuth - min_azimuth > 0.08:
            raise ValueError('near-field exclusion azimuth span exceeds 0.08 rad')
        if not -math.pi / 2 <= min_elevation < max_elevation <= math.pi / 2:
            raise ValueError('near-field exclusion elevation bounds are invalid')
        if max_elevation - min_elevation > 0.06:
            raise ValueError('near-field exclusion elevation span exceeds 0.06 rad')
        exclusions.append(CoMovingNearFieldExclusion(
            channel_name=channel_name,
            min_range_m=min_range,
            max_range_m=max_range,
            min_azimuth_rad=min_azimuth,
            max_azimuth_rad=max_azimuth,
            min_elevation_rad=min_elevation,
            max_elevation_rad=max_elevation,
        ))
    return tuple(exclusions)


def filter_co_moving_near_field_detections(
    channel_name: str,
    detections: Iterable[CarlaRadarDetection],
    exclusions: Sequence[CoMovingNearFieldExclusion],
    max_abs_velocity_mps: float,
) -> tuple[tuple[CarlaRadarDetection, ...], int]:
    """Remove only detections matching a tight, near-zero-Doppler raw cell.

    Range alone is intentionally insufficient.  A moving target in the same
    location is retained, as are all detections outside the configured raw
    azimuth/elevation cell.
    """
    velocity_limit = float(max_abs_velocity_mps)
    if (
        not math.isfinite(velocity_limit)
        or velocity_limit < 0.0
        or velocity_limit > 0.10
    ):
        raise ValueError(
            'co-moving near-field max_abs_velocity_mps must be in [0, 0.10]'
        )
    channel = str(channel_name).strip().lower()
    relevant = tuple(
        exclusion
        for exclusion in exclusions
        if exclusion.channel_name == channel
    )
    kept = []
    filtered_count = 0
    for detection in detections:
        matched = (
            math.isfinite(detection.range_m)
            and math.isfinite(detection.velocity_mps)
            and math.isfinite(detection.azimuth_rad)
            and math.isfinite(detection.elevation_rad)
            and abs(detection.velocity_mps) <= velocity_limit
            and any(
                exclusion.min_range_m <= detection.range_m <= exclusion.max_range_m
                and exclusion.min_azimuth_rad
                <= detection.azimuth_rad
                <= exclusion.max_azimuth_rad
                and exclusion.min_elevation_rad
                <= detection.elevation_rad
                <= exclusion.max_elevation_rad
                for exclusion in relevant
            )
        )
        if matched:
            filtered_count += 1
        else:
            kept.append(detection)
    return tuple(kept), filtered_count


def select_nearest_range(
    detections_m: Iterable[float],
    min_range_m: float,
    max_range_m: float,
    no_return_epsilon_m: float = NO_RETURN_EPSILON_M,
) -> RadarRangeSelection:
    """
    Reduce a CARLA radar cloud to CAMROD's nearest scalar range.

    A cloud with no valid point uses the same ``max_range + 1 mm`` heartbeat
    as the physical SEN0592 driver.  Consumers therefore distinguish a fresh
    no-target sample from a stale sensor without painting a false obstacle.
    """
    minimum = float(min_range_m)
    maximum = float(max_range_m)
    epsilon = float(no_return_epsilon_m)
    if not math.isfinite(minimum) or minimum < 0.0:
        raise ValueError('min_range_m must be finite and non-negative')
    if not math.isfinite(maximum) or maximum <= minimum:
        raise ValueError('max_range_m must be finite and greater than min_range_m')
    if not math.isfinite(epsilon) or epsilon <= 0.0:
        raise ValueError('no_return_epsilon_m must be finite and positive')

    count = 0
    valid = []
    for detection in detections_m:
        count += 1
        try:
            value = float(detection)
        except (TypeError, ValueError):
            continue
        if math.isfinite(value) and minimum <= value <= maximum:
            valid.append(value)

    if not valid:
        return RadarRangeSelection(
            range_m=maximum + epsilon,
            input_detection_count=count,
            valid_detection_count=0,
            has_target=False,
        )
    return RadarRangeSelection(
        range_m=min(valid),
        input_detection_count=count,
        valid_detection_count=len(valid),
        has_target=True,
    )


def build_channel_specs(
    names: Sequence[str],
    input_topics: Sequence[str],
    output_topics: Sequence[str],
    standard_output_topics: Sequence[str],
    frame_ids: Sequence[str],
    min_ranges_m: Sequence[float],
    max_ranges_m: Sequence[float],
    field_of_views_rad: Sequence[float],
) -> tuple[RadarChannelSpec, ...]:
    """Validate parallel ROS parameter arrays and return immutable specs."""
    vectors = {
        'input_topics': input_topics,
        'output_topics': output_topics,
        'standard_output_topics': standard_output_topics,
        'frame_ids': frame_ids,
        'min_ranges_m': min_ranges_m,
        'max_ranges_m': max_ranges_m,
        'field_of_views_rad': field_of_views_rad,
    }
    size = len(names)
    if size == 0:
        raise ValueError('at least one radar channel is required')
    for label, values in vectors.items():
        if len(values) != size:
            raise ValueError(
                f'{label} must contain {size} values, got {len(values)}'
            )

    specs = []
    seen_names = set()
    seen_inputs = set()
    seen_outputs = set()
    for index, raw_name in enumerate(names):
        name = str(raw_name).strip()
        input_topic = str(input_topics[index]).strip()
        output_topic = str(output_topics[index]).strip()
        standard_topic = str(standard_output_topics[index]).strip()
        frame_id = str(frame_ids[index]).strip()
        if not name or name in seen_names:
            raise ValueError(f'radar channel names must be non-empty and unique: {name!r}')
        if not input_topic.startswith('/') or input_topic in seen_inputs:
            raise ValueError(
                f'radar input topics must be unique absolute names: {input_topic!r}'
            )
        if (
            not output_topic.startswith('/')
            or not standard_topic.startswith('/')
            or output_topic in seen_outputs
            or standard_topic in seen_outputs
            or output_topic == standard_topic
        ):
            raise ValueError('radar output topics must be unique absolute names')
        if not frame_id:
            raise ValueError(f'frame id is empty for radar channel {name}')

        minimum = float(min_ranges_m[index])
        maximum = float(max_ranges_m[index])
        field_of_view = float(field_of_views_rad[index])
        # Reuse the scalar mapper's range validation without retaining a value.
        select_nearest_range((), minimum, maximum)
        if not math.isfinite(field_of_view) or not 0.0 < field_of_view <= math.pi:
            raise ValueError(
                f'field of view for {name} must be finite and in (0, pi]'
            )

        seen_names.add(name)
        seen_inputs.add(input_topic)
        seen_outputs.update((output_topic, standard_topic))
        specs.append(RadarChannelSpec(
            name=name,
            input_topic=input_topic,
            output_topic=output_topic,
            standard_output_topic=standard_topic,
            frame_id=frame_id,
            min_range_m=minimum,
            max_range_m=maximum,
            field_of_view_rad=field_of_view,
        ))
    return tuple(specs)
