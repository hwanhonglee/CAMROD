import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_dir = get_package_share_directory('camrod_voice')
    cfg = lambda f: os.path.join(pkg_dir, 'config', f)  # noqa: E731

    voice_namespace = LaunchConfiguration('voice_namespace')
    enable_voice_adapter = LaunchConfiguration('enable_voice_adapter')
    locale = LaunchConfiguration('locale')

    return LaunchDescription([
        DeclareLaunchArgument(
            'voice_namespace',
            default_value='voice',
            description='Namespace for voice module nodes',
        ),
        DeclareLaunchArgument(
            'enable_voice_adapter',
            default_value='true',
            description='Enable voice event adapter node (system-event → AudioRequest bridge)',
        ),
        DeclareLaunchArgument(
            'locale',
            default_value='ko-KR',
            description='Audio locale directory under resource/audio/',
        ),

        GroupAction([
            PushRosNamespace(voice_namespace),

            Node(
                package='camrod_voice',
                executable='voice_announcer_node',
                name='voice_announcer',
                output='screen',
                # Jetson runs PipeWire's PulseAudio compatibility server. Pin
                # SDL to that session so it follows the selected BT sink.
                additional_env={'SDL_AUDIODRIVER': 'pulseaudio'},
                # The launch argument wins over the file default for locale.
                parameters=[cfg('voice_announcer.yaml'), {'locale': locale}],
            ),

            Node(
                package='camrod_voice',
                executable='voice_event_adapter_node',
                name='voice_event_adapter',
                output='screen',
                condition=IfCondition(enable_voice_adapter),
                parameters=[cfg('voice_event_adapter.yaml')],
            ),
        ]),
    ])
