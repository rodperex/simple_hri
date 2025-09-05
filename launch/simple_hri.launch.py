from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_hri',
            executable='stt_service_local',
            name='stt_service_node',
            output='screen'
        ),
        # Node(
        #     package='simple_hri',
        #     executable='stt_service_local',
        #     name='stt_service_node',
        #     output='screen'
        # ),
        Node(
            package='simple_hri',
            executable='tts_service',
            name='tts_service_node',
            output='screen'
        ),
    ])
