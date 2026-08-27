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
        DeclareLaunchArgument("front_frame_id", default_value="camera_front"),
        DeclareLaunchArgument("rear_frame_id", default_value="camera_rear"),
        DeclareLaunchArgument("lidar_frame_id", default_value="lidar_link"),
        DeclareLaunchArgument("preserve_lidar_frame", default_value="false"),
        DeclareLaunchArgument(
            "launch_image_compression", default_value="true"
        ),
        DeclareLaunchArgument("jpeg_quality", default_value="80"),
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
                "use_sim_time": False,
            }],
        ),
    ])


if __name__ == "__main__":
    generate_launch_description()
