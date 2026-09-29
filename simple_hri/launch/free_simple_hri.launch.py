from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
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
        description='Language code for TTS service (e.g., spa, eng)'
    )

    audio_player_arg = DeclareLaunchArgument(
        'audio_player',
        default_value='',
        description='Command used to play audio (e.g. "aplay -q", "pw-play"). Empty = auto'
    )

    def start_interaction_services(context):

        if LaunchConfiguration('run_interaction_services').perform(context) == 'true':

            language = LaunchConfiguration('tts_lang').perform(context)
            audio_player = LaunchConfiguration('audio_player').perform(context)
            
            tts_speaks_val = LaunchConfiguration('tts_speaks').perform(context)
            play_sound_param = (tts_speaks_val.lower() == 'true')

            interaction_nodes = GroupAction([
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
                    output='screen',
                    parameters=[
                        {'lang_code': language},
                        {'play_sound': play_sound_param},
                        {'audio_player': audio_player}
                    ]
                ),
                Node(
                    package='simple_hri',
                    executable='extract_service_hugg',
                    name='extract_service_node',
                    output='screen'
                ),
                Node(
                    package='simple_hri',
                    executable='yesno_service_local',
                    name='yesno_service_node',
                    output='screen'
                )
            ])
            return [interaction_nodes]
        return []
    
    def start_audio_services(context):

        if LaunchConfiguration('run_interaction_services').perform(context) == 'false':
            audio_player = LaunchConfiguration('audio_player').perform(context)
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
                    output='screen',
                    parameters=[{'audio_player': audio_player}]
                )
            ])
            return [audio_nodes]
        return []
    
    ld = LaunchDescription()

    ld.add_action(run_interaction_arg)
    ld.add_action(tts_speaks_arg_decl)
    ld.add_action(tts_lang_arg)
    ld.add_action(audio_player_arg)

    ld.add_action(OpaqueFunction(function=start_interaction_services))
    ld.add_action(OpaqueFunction(function=start_audio_services))
    
    return ld