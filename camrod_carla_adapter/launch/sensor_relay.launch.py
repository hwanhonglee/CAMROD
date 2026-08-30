"""Relay the CARLA Ranger sensor suite to CAMROD topic names."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _role_topic(suffix):
    return [
        TextSubstitution(text="/carla/"),
        LaunchConfiguration("role_name"),
        TextSubstitution(text=suffix),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("role_name", default_value="ego_vehicle"),
        DeclareLaunchArgument(
            "front_image_input", default_value=_role_topic("/rgb_view/image")
        ),
        DeclareLaunchArgument(
            "front_info_input",
            default_value=_role_topic("/rgb_view/camera_info"),
        ),
        DeclareLaunchArgument(
            "rear_image_input", default_value=_role_topic("/rgb_rear/image")
        ),
        DeclareLaunchArgument(
            "rear_info_input",
            default_value=_role_topic("/rgb_rear/camera_info"),
        ),
        DeclareLaunchArgument(
            "lidar_input", default_value=_role_topic("/lidar_front")
        ),
        DeclareLaunchArgument(
            "lidar_output",
            default_value="/sensing/lidar/vanjee/points_raw",
        ),
        DeclareLaunchArgument("front_frame_id", default_value="camera_front"),
        DeclareLaunchArgument("rear_frame_id", default_value="camera_rear"),
        DeclareLaunchArgument("lidar_frame_id", default_value="lidar_link"),
        DeclareLaunchArgument("preserve_lidar_frame", default_value="false"),
        DeclareLaunchArgument(
            "launch_image_compression", default_value="true"
        ),
        DeclareLaunchArgument("jpeg_quality", default_value="80"),
        DeclareLaunchArgument(
            "compressed_image_max_rate_hz",
            default_value="10.0",
            description=(
                "Maximum wall-clock JPEG publication rate per active camera"
            ),
        ),
        DeclareLaunchArgument(
            "raw_image_max_rate_hz",
            default_value="10.0",
            description=(
                "Maximum wall-clock canonical raw publication rate per "
                "camera; frames are skipped entirely without subscribers"
            ),
        ),
        Node(
            package="camrod_carla_adapter",
            executable="carla_sensor_relay",
            name="carla_sensor_relay",
            output="screen",
            parameters=[{
                "front_image_input": LaunchConfiguration("front_image_input"),
                "front_info_input": LaunchConfiguration("front_info_input"),
                "rear_image_input": LaunchConfiguration("rear_image_input"),
                "rear_info_input": LaunchConfiguration("rear_info_input"),
                "lidar_input": LaunchConfiguration("lidar_input"),
                "lidar_output": LaunchConfiguration("lidar_output"),
                "front_frame_id": LaunchConfiguration("front_frame_id"),
                "rear_frame_id": LaunchConfiguration("rear_frame_id"),
                "lidar_frame_id": LaunchConfiguration("lidar_frame_id"),
                "preserve_lidar_frame": LaunchConfiguration(
                    "preserve_lidar_frame"
                ),
                "publish_compressed_images": ParameterValue(
                    LaunchConfiguration("launch_image_compression"),
                    value_type=bool,
                ),
                "jpeg_quality": ParameterValue(
                    LaunchConfiguration("jpeg_quality"), value_type=int
                ),
                "compressed_image_max_rate_hz": ParameterValue(
                    LaunchConfiguration("compressed_image_max_rate_hz"),
                    value_type=float,
                ),
                "raw_image_max_rate_hz": ParameterValue(
                    LaunchConfiguration("raw_image_max_rate_hz"),
                    value_type=float,
                ),
                "use_sim_time": False,
            }],
        ),
    ])


if __name__ == "__main__":
    generate_launch_description()
