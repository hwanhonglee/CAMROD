from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('camrod_sensing')

    default_config_file = os.path.join(pkg_dir, 'config', 'camera', 'camera_params.yaml')
    cyclonedds_config = os.path.join(pkg_dir, 'config', 'camera', 'cyclonedds.xml')

    config_file_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=default_config_file,
        description='Path to camera configuration file'
    )
    device_path_arg = DeclareLaunchArgument(
        'device_path',
        default_value='/dev/video0',
        description='Camera device path'
    )
    module_namespace_arg = DeclareLaunchArgument(
        'module_namespace',
        default_value='camera',
        description='Camera module namespace'
    )
    roudi_config = os.path.join(pkg_dir, 'config', 'camera', 'roudi_config.toml')

    # Iceoryx shared memory daemon — enables zero-copy DDS for same-machine subscribers
    # Custom config adds 8MB mempool for 1920x1080 BGR images (~6.2MB each)
    iox_roudi = ExecuteProcess(
        cmd=['iox-roudi', '-l', 'warning', '-c', roudi_config],
        output='screen',
        name='iox-roudi',
    )

    camera_node = Node(
        package='camrod_sensing',
        executable='camera_publisher_node',
        name='camera_publisher',
        namespace=LaunchConfiguration('module_namespace'),
        output='screen',
        parameters=[
            LaunchConfiguration('camera_params_file'),
            {
                'device_path': LaunchConfiguration('device_path'),
            }
        ],
        remappings=[
            ('~/image_rect/compressed', 'image_rect/compressed'),
            ('~/camera_info', 'camera_info'),
        ],
        additional_env={
            'CYCLONEDDS_URI': cyclonedds_config,
        },
    )

    return LaunchDescription([
        config_file_arg,
        device_path_arg,
        module_namespace_arg,
        iox_roudi,
        camera_node,
    ])
