from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_hri',
            executable='stt_service_local',
            name='stt_service_node',
            output='screen'
        ),
        Node(
            package='simple_hri',
            executable='tts_service_local',
            name='tts_service_node',
            output='screen'
        ),
        Node(
            package='simple_hri',
            executable='extract_service_local',
            name='extract_service_node',
            output='screen'
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                FindPackageShare('sound_play'), '/launch/soundplay_node.launch.xml'
            ])
        ),
    ])
