from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    run_interaction_arg = DeclareLaunchArgument(
        'run_interaction_services',
        default_value='true',
        description='Set to "true" every interaction service will be launched'
    )

    tts_speaks_arg_decl = DeclareLaunchArgument(
        'tts_speaks',
        default_value='true',
        description='Set to "true" to enable sound output from the TTS node directly'
    )

    tts_lang_arg = DeclareLaunchArgument(
        'tts_lang',
        default_value='spa',
        description='Language code for TTS service (e.g., en_US, es_ES)'
    )

    sound_play_arg = DeclareLaunchArgument(
        'start_sound_play',
        default_value='true',
        description='Set to "true" to use sound_play for audio output'
    )

    def start_interaction_services(context):

        if LaunchConfiguration('run_interaction_services').perform(context) == 'true':

            language = LaunchConfiguration('tts_lang').perform(context)
            
            tts_speaks_val = LaunchConfiguration('tts_speaks').perform(context)
            play_sound_param = (tts_speaks_val.lower() == 'true')

            interaction_nodes = GroupAction([
                Node(
                    package='simple_hri',
                    executable='stt_service',
                    name='stt_service_node',
                    output='screen'
                ),
                Node(
                    package='simple_hri',
                    executable='tts_service',
                    name='tts_service_node',
                    output='screen',
                    parameters=[
                        {'language': language},
                        {'play_sound': play_sound_param} 
                    ]
                ),
                Node(
                    package='simple_hri',
                    executable='extract_service',
                    name='extract_service_node',
                    output='screen'
                ),
                Node(
                    package='simple_hri',
                    executable='yesno_service',
                    name='yesno_service_node',
                    output='screen'
                )
            ])
            return [interaction_nodes]
        return []
    
    def start_audio_services(context):

        if LaunchConfiguration('run_interaction_services').perform(context) == 'false':
            audio_nodes = GroupAction([
                Node(
                    package='simple_hri',
                    executable='audio_service',
                    name='audio_service_node',
                    output='screen'
                ),
                Node(
                    package='simple_hri',
                    executable='audio_file_player',
                    name='audio_file_player_node',
                    output='screen'
                )
            ])
            return [audio_nodes]
        return []
    

    def start_sound_play(context):
        if LaunchConfiguration('start_sound_play').perform(context) == 'true':
            sound_play_nodes = GroupAction([
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource([
                        FindPackageShare('sound_play'), '/launch/soundplay_node.launch.xml'
                    ])
                )
            ])
            return [sound_play_nodes]
        return []
    
    ld = LaunchDescription()

    ld.add_action(run_interaction_arg)
    ld.add_action(tts_speaks_arg_decl)
    ld.add_action(tts_lang_arg)
    ld.add_action(sound_play_arg)

    ld.add_action(OpaqueFunction(function=start_interaction_services))
    ld.add_action(OpaqueFunction(function=start_audio_services))
    ld.add_action(OpaqueFunction(function=start_sound_play))
    
    return ld