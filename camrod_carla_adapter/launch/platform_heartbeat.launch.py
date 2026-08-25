"""Publish controllable raw Ranger/BMS status for CARLA software tests."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
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
                    LaunchConfiguration("publish_rate_hz"), value_type=float
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
