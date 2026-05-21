import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_param = os.path.join(
        get_package_share_directory('camrod_perception'),
        'config', 'perception_params.yaml',
    )

    # Resolve model path from camrod_perception share (models/ installed by CMakeLists).
    # Resolve labels path from yolov9mit_ros share (built in camera_lidar_fusion/yolo_ws).
    try:
        yolo_ros_share = get_package_share_directory('yolov9mit_ros')
        default_model = os.path.join(yolo_ros_share, 'models', 'v9-s.vec2box.sim.engine')
        default_labels = os.path.join(yolo_ros_share, 'labels', 'coco_names.txt')
    except Exception:
        default_model = ''
        default_labels = ''

    return LaunchDescription([
        DeclareLaunchArgument('module_namespace',      default_value='perception'),
        DeclareLaunchArgument('perception_param_file', default_value=default_param),
        DeclareLaunchArgument('enable_yolo',           default_value='true'),
        DeclareLaunchArgument('yolo_model_path',       default_value=default_model),
        DeclareLaunchArgument('yolo_labels_path',      default_value=default_labels),

        Node(
            package='yolov9mit_ros',
            executable='yolov9mit_ros_node',
            name='yolov9mit',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('perception_param_file'),
                {
                    'model_path':       LaunchConfiguration('yolo_model_path'),
                    'class_label_path': LaunchConfiguration('yolo_labels_path'),
                    'transport_hint':   'raw',
                },
            ],
            condition=IfCondition(LaunchConfiguration('enable_yolo')),
        ),
    ])
