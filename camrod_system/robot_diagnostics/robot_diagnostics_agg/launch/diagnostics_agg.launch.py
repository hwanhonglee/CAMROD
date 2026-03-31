from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_diagnostics_agg'), 'config', 'diagnostics_config.yaml']
    )

    aggregator = Node(
        package='robot_diagnostics_agg',
        executable='aggregator_node',
        name='diagnostics_agg',
        parameters=[{'config_file': config}],
        output='screen',
    )

    return LaunchDescription([aggregator])
