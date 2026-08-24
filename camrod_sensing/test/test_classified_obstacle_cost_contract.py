"""Lock marker-free classified LiDAR cost routing across package boundaries."""

from pathlib import Path

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
SENSING_CONFIG = SRC_ROOT / "camrod_sensing/config/lidar/cost_grid.yaml"
BRINGUP_SENSING_CONFIG = (
    SRC_ROOT / "camrod_bringup/config/sensing/lidar/cost_grid.yaml"
)
PERCEPTION_CONFIG = SRC_ROOT / "camrod_perception/config/perception_params.yaml"
BRINGUP_PERCEPTION_CONFIG = (
    SRC_ROOT / "camrod_bringup/config/perception/perception_params.yaml"
)
LIDAR_GRID_SOURCE = SRC_ROOT / "camrod_sensing/src/lidar_cost_grid_node.cpp"
FUSION_SOURCE = SRC_ROOT / "camrod_perception/src/obstacle_fusion_node.cpp"


def _parameters(path: Path, node_name: str) -> dict:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    return document[node_name]["ros__parameters"]


def test_safety_grid_configs_are_identical_and_class_cloud_only() -> None:
    """Full bringup must deploy the package's semantic cloud-only contract."""
    assert BRINGUP_SENSING_CONFIG.read_bytes() == SENSING_CONFIG.read_bytes()

    parameters = _parameters(
        SENSING_CONFIG, "/sensing/lidar/lidar_cost_grid"
    )
    assert parameters["input_topic"] == "/perception/obstacles"
    assert parameters["input_topics"] == ["/perception/obstacles"]
    assert parameters["raw_lidar_cost_enabled"] is False
    assert not any(key.startswith("perception_marker") for key in parameters)


def test_lidar_grid_has_no_marker_to_cost_implementation() -> None:
    """A launch override must not be able to restore MarkerArray safety cost."""
    source = LIDAR_GRID_SOURCE.read_text(encoding="utf-8")
    for forbidden in (
        "perception_marker",
        "MarkerArray",
        "markMarkerInput",
        "transformMarkerPoint",
    ):
        assert forbidden not in source


def test_fusion_applies_one_class_and_geometry_filter_to_all_outputs() -> None:
    """Cloud, 3D markers, and Euclidean class association share one guard."""
    assert BRINGUP_PERCEPTION_CONFIG.read_bytes() == PERCEPTION_CONFIG.read_bytes()
    parameters = _parameters(PERCEPTION_CONFIG, "/perception/obstacle_fusion")
    assert parameters["output_topic"] == "/perception/obstacles"
    assert parameters["unknown_class_labels"] == ["", "?", "unknown"]

    source = FUSION_SOURCE.read_text(encoding="utf-8")
    assert source.count("classifiedDetectionLabel(d2)") == 3
    assert "camrod_perception::HasValidDetectionBox(" in source
    assert "camrod_perception::IsClassifiedDetection(" in source

    # Unknown Euclidean clusters remain visible for diagnostics, but the
    # rasterizer above has no subscription to this visualization output.
    assert 'std::vector<std::string> labels(cls.size(), "unknown")' in source
    assert "pub_euclidean_->publish(out)" in source
