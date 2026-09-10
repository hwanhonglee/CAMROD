"""Launch the production CAMROD UI against the interactive runtime simulator."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    tester_share = get_package_share_directory("camrod_ui_tester")
    ui_share = get_package_share_directory("camrod_ui")
    camping_sites = os.path.join(tester_share, "config", "camping_sites_test.yaml")
    drop_zones = os.path.join(tester_share, "config", "drop_zones_test.yaml")
    scenarios = os.path.join(tester_share, "config", "scenarios.yaml")
    control_panel = os.path.join(tester_share, "assets", "control_panel")

    arguments = [
        DeclareLaunchArgument("mode", default_value="closed_loop"),
        DeclareLaunchArgument("initial_scenario", default_value="ready"),
        DeclareLaunchArgument("speed_scale", default_value="1.0"),
        DeclareLaunchArgument("route_duration_s", default_value="12.0"),
        DeclareLaunchArgument("maneuver_duration_s", default_value="2.0"),
        DeclareLaunchArgument("ui_port", default_value="8010"),
        DeclareLaunchArgument("guest_port", default_value="8012"),
        DeclareLaunchArgument("simulator_port", default_value="8020"),
        DeclareLaunchArgument("enable_guest_ui", default_value="true"),
        DeclareLaunchArgument("enable_operator_ui_window", default_value="false"),
        DeclareLaunchArgument("allow_default_ros_domain", default_value="false"),
    ]

    simulator = Node(
        package="camrod_ui_tester",
        executable="ui_simulator",
        name="ui_simulator",
        output="screen",
        parameters=[
            os.path.join(tester_share, "config", "simulator.yaml"),
            {
                "mode": LaunchConfiguration("mode"),
                "initial_scenario": LaunchConfiguration("initial_scenario"),
                "speed_scale": ParameterValue(LaunchConfiguration("speed_scale"), value_type=float),
                "route_duration_s": ParameterValue(LaunchConfiguration("route_duration_s"), value_type=float),
                "maneuver_duration_s": ParameterValue(LaunchConfiguration("maneuver_duration_s"), value_type=float),
                "control_port": ParameterValue(LaunchConfiguration("simulator_port"), value_type=int),
                # The reset control must release the backend mission it does
                # not own, so it follows ui_port rather than a fixed default.
                "backend_stop_url": PythonExpression([
                    "'http://127.0.0.1:", LaunchConfiguration("ui_port"), "/ui/stop'"
                ]),
                "allow_default_ros_domain": ParameterValue(LaunchConfiguration("allow_default_ros_domain"), value_type=bool),
                "camping_sites_yaml": camping_sites,
                "drop_zones_yaml": drop_zones,
                "scenarios_yaml": scenarios,
                "control_panel_dir": control_panel,
            },
        ],
    )

    ranger_parameter_stub = Node(
        package="camrod_ui_tester",
        executable="ranger_parameter_stub",
        name="ranger_base_node",
        output="screen",
        parameters=[{"steering_transition_rate_radps": 1.0}],
    )

    ui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ui_share, "launch", "ui.launch.py")),
        launch_arguments={
            "ui_host": "127.0.0.1",
            "ui_port": LaunchConfiguration("ui_port"),
            "guest_host": "127.0.0.1",
            "guest_port": LaunchConfiguration("guest_port"),
            "enable_ui_guest": LaunchConfiguration("enable_guest_ui"),
            "enable_operator_ui_window": LaunchConfiguration("enable_operator_ui_window"),
            "operator_ui_window_url": PythonExpression([
                "'http://127.0.0.1:", LaunchConfiguration("ui_port"), "'"
            ]),
            "enable_operator_telemetry": "true",
            "camping_sites_yaml": camping_sites,
            "drop_zones_yaml": drop_zones,
            "manual_return_preempt_hold_s": "0.5",
            "charging_departure_delay_s": "2.0",
            "redock_require_can_control_mode": "false",
        }.items(),
    )

    links = LogInfo(
        msg=[
            "[camrod_ui_tester] Robot UI http://127.0.0.1:", LaunchConfiguration("ui_port"),
            " | Guest UI http://127.0.0.1:", LaunchConfiguration("guest_port"),
            " | Simulator http://127.0.0.1:", LaunchConfiguration("simulator_port"),
        ]
    )

    return LaunchDescription(arguments + [simulator, ranger_parameter_stub, ui, links])
