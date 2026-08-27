"""Publish controllable raw Ranger/BMS status for CARLA software tests."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _prefer_explicit_legacy_rate(primary_argument):
    """Use the deprecated generic rate only when it is explicitly non-empty."""
    return PythonExpression([
        "'",
        LaunchConfiguration("publish_rate_hz"),
        "'.strip() or '",
        LaunchConfiguration(primary_argument),
        "'",
    ])


def generate_launch_description():
    resolved_publish_rate = _prefer_explicit_legacy_rate(
        "platform_heartbeat_publish_rate_hz"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "platform_heartbeat_publish_rate_hz",
            default_value="5.0",
            description="Platform heartbeat publish rate (Hz)",
        ),
        DeclareLaunchArgument(
            "publish_rate_hz",
            default_value="",
            description=(
                "Deprecated alias for platform_heartbeat_publish_rate_hz; "
                "a non-empty value takes precedence"
            ),
        ),
        DeclareLaunchArgument("initial_soc", default_value="0.8"),
        DeclareLaunchArgument("initial_charging", default_value="false"),
        DeclareLaunchArgument("initial_estop", default_value="false"),
        DeclareLaunchArgument("initial_error_code", default_value="0"),
        Node(
            package="camrod_carla_adapter",
            executable="carla_platform_heartbeat",
            name="carla_platform_heartbeat",
            output="screen",
            parameters=[{
                "publish_rate_hz": ParameterValue(
                    resolved_publish_rate, value_type=float
                ),
                "initial_soc": ParameterValue(
                    LaunchConfiguration("initial_soc"), value_type=float
                ),
                "initial_charging": ParameterValue(
                    LaunchConfiguration("initial_charging"), value_type=bool
                ),
                "initial_estop": ParameterValue(
                    LaunchConfiguration("initial_estop"), value_type=bool
                ),
                "initial_error_code": ParameterValue(
                    LaunchConfiguration("initial_error_code"), value_type=int
                ),
                "use_sim_time": False,
            }],
        ),
    ])


if __name__ == "__main__":
    generate_launch_description()
