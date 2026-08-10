"""Lock Nav2 composition without absorbing the final control safety gate."""

from pathlib import Path

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
NAV2_LAUNCH = SRC_ROOT / "camrod_planning" / "launch" / "nav2_lanelet.launch.py"
PLANNING_LAUNCH = SRC_ROOT / "camrod_planning" / "launch" / "planning.launch.py"
BRINGUP_IMPL = SRC_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
BRINGUP_DEFAULTS = (
    SRC_ROOT / "camrod_bringup" / "config" / "bringup" / "launch_defaults.yaml"
)


def test_nav2_component_set_and_standalone_fallback_are_both_present() -> None:
    """Maintained core servers compose while vendor servers stay standalone."""
    source = NAV2_LAUNCH.read_text(encoding="utf-8")

    for plugin in (
        "nav2_planner::PlannerServer",
        "nav2_controller::ControllerServer",
    ):
        assert f"plugin='{plugin}'" in source
    for plugin in (
        "nav2_smoother::SmootherServer",
        "behavior_server::BehaviorServer",
        "nav2_bt_navigator::BtNavigator",
        "nav2_lifecycle_manager::LifecycleManager",
    ):
        assert f"plugin='{plugin}'" not in source

    for executable in (
        "planner_server",
        "controller_server",
        "smoother_server",
        "behavior_server",
        "bt_navigator",
        "lifecycle_manager",
    ):
        assert f"executable='{executable}'" in source

    assert "package='camrod_runtime'" in source
    assert "executable='scoped_component_container_mt'" in source
    # HH_260805 - Humble permits intra-process only for volatile publishers;
    # Nav2's transient-local costmap/route endpoints require DDS delivery.
    assert "'use_intra_process_comms': False" in source
    # HH_260805 - Internal costmap child nodes must receive the center-frame
    # rewrite before component construction, not only a parent-node override.
    assert "nav2_frame_rewrites = {" in source
    assert source.count("nav2_frame_rewrites") >= 8
    assert "parameters=nav2_param_chain" in source
    assert "namespace=module_namespace" in source
    assert source.count("condition=UnlessCondition(use_nav2_container)") == 2
    assert "package='camrod_control'" not in source


def test_lifecycle_manager_standalone_exit_avoids_humble_teardown_race() -> None:
    source = (
        SRC_ROOT
        / "camrod_planning"
        / "external"
        / "nav2_lifecycle_manager"
        / "src"
        / "main.cpp"
    ).read_text(encoding="utf-8")

    assert "node.reset();" in source
    assert "std::fflush(nullptr);" in source
    assert "std::_Exit(EXIT_SUCCESS);" in source


def test_composed_nav2_core_uses_the_container_owned_context() -> None:
    """Embedded threads and TF callbacks must not consult the global context."""
    planning = SRC_ROOT / "camrod_planning" / "external"
    costmap = (
        planning / "nav2_costmap_2d" / "src" / "costmap_2d_ros.cpp"
    ).read_text(encoding="utf-8")
    planner = (
        planning / "nav2_planner" / "src" / "planner_server.cpp"
    ).read_text(encoding="utf-8")
    controller = (
        planning / "nav2_controller" / "src" / "controller_server.cpp"
    ).read_text(encoding="utf-8")
    action_server = (
        planning / "nav2_util" / "include" / "nav2_util"
        / "simple_action_server.hpp"
    ).read_text(encoding="utf-8")

    assert costmap.count(
        "rclcpp::ok(get_node_base_interface()->get_context())"
    ) == 3
    assert "tf_options.callback_group = callback_group_" in costmap
    assert "tf_static_options.callback_group = callback_group_" in costmap
    assert "tf_buffer_->setUsingDedicatedThread(true)" in costmap
    assert "rclcpp::ok(get_node_base_interface()->get_context())" in planner
    assert "while (rclcpp::ok(context))" in controller
    assert "rclcpp::ok(node_base_interface_->get_context())" in action_server


def test_bringup_defaults_nav2_container_and_forwards_fallback_switch() -> None:
    """The validated default and fallback switch must reach both launch layers."""
    defaults = yaml.safe_load(BRINGUP_DEFAULTS.read_text(encoding="utf-8"))
    # HH_260805 - Scoped context teardown passed three final full-sim runs.
    assert defaults["bringup"]["planning"]["use_nav2_container"] is True

    bringup = BRINGUP_IMPL.read_text(encoding="utf-8")
    planning = PLANNING_LAUNCH.read_text(encoding="utf-8")
    assert "'use_nav2_container': lc['use_nav2_container']" in bringup
    assert "cfg_get(launch_cfg, 'planning/use_nav2_container', True)" in bringup
    assert "DeclareLaunchArgument('use_nav2_container', default_value='true')" in planning
    assert "'use_nav2_container'," in planning
    assert planning.count("'nav2_controller_plugins_param_file'") >= 2
