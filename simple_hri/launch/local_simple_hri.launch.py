import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    
    tts_talks = True

    if not tts_talks:
        param = 'true'
    else:
        param = 'false'
    # Argument to toggle audio nodes on/off
    run_audio_service_arg = DeclareLaunchArgument(
        'run_audio_service',
        default_value=param,
        description='Set to "true" to launch audio service, player, and sound_play'
    )

    # Argument for TTS Language
    tts_lang_arg = DeclareLaunchArgument(
        'tts_lang',
        default_value='spa',
        description='Language code for TTS service (e.g., en_US, es_ES)'
    )

    # Argument to toggle sound play node on/off
    run_sound_play_arg = DeclareLaunchArgument(
        'run_sound_play',
        default_value='true',
        description='Set to "false" to disable the sound play node'
    )

    
    # Create variables to hold the values of the arguments
    should_run_audio_service = LaunchConfiguration('run_audio_service')
    should_run_sound_play = LaunchConfiguration('run_sound_play')
    tts_language_config = LaunchConfiguration('tts_lang')

    stt_node = Node(
        package='simple_hri',
        executable='stt_service_local',
        name='stt_service_node',
        output='screen'
    )

    tts_node = Node(
        package='simple_hri',
        executable='tts_service_local',
        name='tts_service_node',
        output='screen',
        parameters=[
            {'language': tts_language_config},
            {'play_sound': tts_talks} # False when using audio service
        ]
    )   

    extract_node = Node(
        package='simple_hri',
        executable='extract_service_local',
        name='extract_service_node',
        output='screen'
    )

    audio_service_node = Node(
        package='simple_hri',
        executable='audio_service',
        name='audio_service_node',
        output='screen',
        condition=IfCondition(should_run_audio_service)
    )

    audio_player_node = Node(
        package='simple_hri',
        executable='audio_file_player',
        name='audio_file_player_node',
        output='screen',
        condition=IfCondition(should_run_audio_service)
    )

    sound_play_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('sound_play'), '/launch/soundplay_node.launch.xml'
        ]),
        condition=IfCondition(should_run_sound_play)
    )

    
    return LaunchDescription([
        # Arguments
        run_audio_service_arg,
        tts_lang_arg,
        run_sound_play_arg,
        
        # Standard Nodes
        stt_node,
        tts_node,
        extract_node,
        
        # Conditional Nodes
        audio_service_node,
        audio_player_node,
        sound_play_launch,
    ])