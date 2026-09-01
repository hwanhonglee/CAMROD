import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def _default_yolo_paths() -> tuple[str, str]:
    try:
        yolo_share = get_package_share_directory("yolov9mit_ros")
        model = os.path.join(
            yolo_share, "models", "epoch74_step151350.vec2box.sim.engine")
        labels = os.path.join(yolo_share, "labels", "coco_names.txt")
        model_override = os.environ.get("YOLOV9_MODEL_PATH", "").strip()
        return model_override or model, labels
    except Exception:
        return os.environ.get("YOLOV9_MODEL_PATH", "").strip(), ""


def generate_launch_description():
    default_camera_params = _pkg_share(
        "camrod_sensing", "config/camera/camera_params.yaml")
    default_perception_params = _pkg_share(
        "camrod_perception", "config/perception_params.yaml")
    default_model, default_labels = _default_yolo_paths()

    return LaunchDescription([
        DeclareLaunchArgument("enable_container", default_value="false"),
        DeclareLaunchArgument("camera_params_file", default_value=default_camera_params),
        DeclareLaunchArgument("perception_param_file", default_value=default_perception_params),
        DeclareLaunchArgument("front_camera_namespace", default_value="/sensing/camera/econ_front"),
        DeclareLaunchArgument("perception_namespace", default_value="perception"),
        DeclareLaunchArgument("yolo_model_path", default_value=default_model),
        DeclareLaunchArgument("yolo_labels_path", default_value=default_labels),

        # HH_260707: Opt-in component path for the front-camera->YOLO hot path.
        # The public node names and topics stay identical to the regular launches.
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container_mt",
            name="camera_yolo_container",
            namespace="",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_container")),
            composable_node_descriptions=[
                ComposableNode(
                    package="camrod_sensing",
                    plugin="camrod::sensing::CameraFrontPublisherNode",
                    name="camera_front_publisher",
                    namespace=LaunchConfiguration("front_camera_namespace"),
                    parameters=[LaunchConfiguration("camera_params_file")],
                    remappings=[
                        ("~/image_rect/compressed", "image_rect/compressed"),
                        ("~/image_raw", "image_raw"),
                        ("~/camera_info", "camera_info"),
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="yolov9mit_ros",
                    plugin="yolov9mit_ros::YOLOV9MIT_Node",
                    name="yolov9mit",
                    namespace=LaunchConfiguration("perception_namespace"),
                    parameters=[
                        LaunchConfiguration("perception_param_file"),
                        {
                            "model_path": LaunchConfiguration("yolo_model_path"),
                            "class_label_path": LaunchConfiguration("yolo_labels_path"),
                            "transport_hint": "compressed",
                        },
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
        ),
    ])
