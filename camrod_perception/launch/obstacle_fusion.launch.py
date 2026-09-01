import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_param = os.path.join(
        get_package_share_directory('camrod_perception'),
        'config',
        'perception_params.yaml',
    )
    default_runtime_override = os.path.join(
        get_package_share_directory('camrod_perception'),
        'config',
        'perception_runtime_profiles',
        'disabled.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='perception'),
        DeclareLaunchArgument('perception_param_file', default_value=default_param),
        DeclareLaunchArgument(
            'perception_runtime_override_param_file',
            default_value=default_runtime_override,
            description=(
                'Final sparse perception overlay; ordinary CAMROD uses an '
                'empty profile'
            ),
        ),

        Node(
            package='camrod_perception',
            executable='obstacle_fusion_node',
            name='obstacle_fusion',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('perception_param_file'),
                LaunchConfiguration('perception_runtime_override_param_file'),
            ],
        ),
    ])
