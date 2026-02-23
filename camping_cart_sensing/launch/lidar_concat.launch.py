from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    topic1_arg = DeclareLaunchArgument(
        'topic1',
        default_value='/os_cloud_node/points',
        description='Ouster point cloud topic'
    )
    topic2_arg = DeclareLaunchArgument(
        'topic2',
        default_value='/vanjee_points760',
        description='Vanjee point cloud topic'
    )
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/sensing/lidar/concatenated/pointcloud',
        description='Concatenated point cloud topic'
    )
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='os_lidar',
        description='Target frame for concatenation'
    )
    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='10',
        description='Approx sync queue size'
    )
    slop_arg = DeclareLaunchArgument(
        'slop',
        default_value='0.05',
        description='Approx sync slop (seconds)'
    )

    # Static TF: os_lidar -> vanjee_lidar
    # x=0.0105, y=0.0, z=-0.2872, pitch=-15deg
    # Quaternion (roll=0, pitch=-15deg, yaw=0):
    # qx=0, qy=-0.130526, qz=0, qw=0.991445
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='os_to_vanjee_static_tf',
        arguments=[
            '0.0105', '0.0', '-0.2872',
            '0', '-0.130526', '0', '0.991445',
            'os_lidar', 'vanjee_lidar'
        ],
        output='screen'
    )

    # Point cloud concat node (Python/C++ whichever you implemented)
    concat_node = Node(
        package='camping_cart_sensing',          # <- 패키지명 맞게 수정 가능
        executable='pointcloud_concat_node',     # <- executable 이름 맞게 수정
        name='pointcloud_concat_node',
        output='screen',
        parameters=[{
            'topic1': LaunchConfiguration('topic1'),
            'topic2': LaunchConfiguration('topic2'),
            'output_topic': LaunchConfiguration('output_topic'),
            'target_frame': LaunchConfiguration('target_frame'),
            'queue_size': LaunchConfiguration('queue_size'),
            'slop': LaunchConfiguration('slop'),
        }]
    )

    return LaunchDescription([
        topic1_arg,
        topic2_arg,
        output_topic_arg,
        target_frame_arg,
        queue_size_arg,
        slop_arg,
        static_tf_node,
        concat_node,
    ])
