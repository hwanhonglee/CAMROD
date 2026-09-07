import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def package_path(package_name: str, relative_path: str) -> str:
    return os.path.join(get_package_share_directory(package_name), relative_path)


def generate_launch_description():
    # Auto mode runs both implementations on private topics. Only the
    # dispatcher may publish shared velocity/service state, after cancellation
    # acknowledgements have transferred ownership to the SOC-selected method.
    mode = LaunchConfiguration("parking_method")

    def private_when_auto(private_topic, standalone_topic):
        return PythonExpression([
            "'", private_topic, "' if '", mode,
            "'.strip().lower() == 'auto' else '", standalone_topic, "'",
        ])
    default_parameter_file = package_path(
        "camrod_control", os.path.join("config", "parking.yaml")
    )
    default_apriltag_parameter_file = package_path(
        "camrod_perception",
        os.path.join("config", "apriltag_parking_detector.yaml"),
    )
    # HH_260807 - Give standalone reverse parking the active map semantics.
    default_drop_zones_yaml = package_path(
        "camrod_map", os.path.join("config", "drop_zones.yaml")
    )

    return LaunchDescription([
        DeclareLaunchArgument("parking_namespace", default_value="parking"),
        DeclareLaunchArgument("parameter_file", default_value=default_parameter_file),
        DeclareLaunchArgument(
            "apriltag_parameter_file",
            default_value=default_apriltag_parameter_file,
        ),
        DeclareLaunchArgument("parking_method", default_value="auto"),
        DeclareLaunchArgument("charging_threshold_percent", default_value="35.0"),
        DeclareLaunchArgument("command_topic", default_value="/control/cmd_vel_raw"),
        DeclareLaunchArgument("vehicle_pose_topic", default_value="/localization/pose"),
        DeclareLaunchArgument("drop_zones_yaml", default_value=default_drop_zones_yaml),
        # Auto policy may request AprilTag later while parked, so its detector
        # must be available before the low-SOC transition or explicit docking.
        DeclareLaunchArgument("launch_apriltag_detector", default_value="true"),
        # HH_260814 - The rear camera node publishes image_rect, so the image_proc
        # fallback stays off unless a raw-only source is being replayed.
        DeclareLaunchArgument("apriltag_launch_rectify", default_value="false"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_path(
                "camrod_perception",
                os.path.join("launch", "apriltag_parking_detector.launch.py"),
            )),
            launch_arguments={
                "parameter_file": LaunchConfiguration("apriltag_parameter_file"),
                "launch_rectify": LaunchConfiguration("apriltag_launch_rectify"),
            }.items(),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("parking_method"),
                "'.strip().lower() in ['apriltag', 'auto'] and '",
                LaunchConfiguration("launch_apriltag_detector"),
                "'.strip().lower() in ['1', 'true', 'yes', 'on']",
            ])),
        ),

        Node(
            package="camrod_control",
            executable="parking_dispatcher_node",
            namespace=LaunchConfiguration("parking_namespace"),
            name="parking_dispatcher",
            output="screen",
            parameters=[LaunchConfiguration("parameter_file"), {
                "command_topic": LaunchConfiguration("command_topic"),
                "charging_threshold_percent": ParameterValue(
                    LaunchConfiguration("charging_threshold_percent"), value_type=float),
            }],
            condition=IfCondition(PythonExpression([
                "'", mode, "'.strip().lower() == 'auto'",
            ])),
        ),

        Node(
            package="camrod_control",
            # HH_260720 - Launch the concrete reverse-parking controller without a legacy alias.
            executable="reverse_parking_controller_node",
            namespace=LaunchConfiguration("parking_namespace"),
            name="reverse_parking_controller",
            output="screen",
            parameters=[
                LaunchConfiguration("parameter_file"),
                {
                    "command_topic": PythonExpression([
                        "'/parking/private/reverse/cmd_vel' if '", mode,
                        "'.strip().lower() == 'auto' else '",
                        LaunchConfiguration("command_topic"), "'",
                    ]),
                    "operation_topic": private_when_auto(
                        "/parking/private/reverse/operation", "/parking/operation"),
                    "status_topic": private_when_auto(
                        "/parking/private/reverse/status", "/parking/reverse_parking_controller/status"),
                    "service_state_topic": private_when_auto(
                        "/parking/private/reverse/service_state", "/service/state"),
                    "diagnostics_topic": private_when_auto(
                        "/parking/private/reverse/diagnostics", "/system/diagnostics"),
                    "vehicle_pose_topic": LaunchConfiguration("vehicle_pose_topic"),
                    "drop_zones_yaml": LaunchConfiguration("drop_zones_yaml"),
                },
            ],
            remappings=[(
                "/parking/reverse_parking_controller/request_operation",
                private_when_auto("/parking/private/reverse/request_operation",
                                  "/parking/reverse_parking_controller/request_operation"),
            )],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("parking_method"), "'.strip().lower() in ['reverse', 'auto']"
            ])),
        ),
        Node(
            package="camrod_control",
            executable="apriltag_parking_controller_node",
            namespace=LaunchConfiguration("parking_namespace"),
            name="apriltag_parking_controller",
            output="screen",
            parameters=[
                LaunchConfiguration("parameter_file"),
                {
                    "command_topic": PythonExpression([
                        "'/parking/private/apriltag/cmd_vel' if '", mode,
                        "'.strip().lower() == 'auto' else '",
                        LaunchConfiguration("command_topic"), "'",
                    ]),
                    "operation_topic": private_when_auto(
                        "/parking/private/apriltag/operation", "/parking/operation"),
                    "status_topic": private_when_auto(
                        "/parking/private/apriltag/status", "/parking/apriltag_parking_controller/status"),
                    "service_state_topic": private_when_auto(
                        "/parking/private/apriltag/service_state", "/service/state"),
                    "diagnostics_topic": private_when_auto(
                        "/parking/private/apriltag/diagnostics", "/system/diagnostics"),
                },
            ],
            remappings=[(
                "/parking/apriltag_parking_controller/request_operation",
                private_when_auto("/parking/private/apriltag/request_operation",
                                  "/parking/apriltag_parking_controller/request_operation"),
            )],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("parking_method"), "'.strip().lower() in ['apriltag', 'auto']"
            ])),
        ),
    ])
