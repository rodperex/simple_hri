# Copyright 2026 Rodrigo Pérez-Rodríguez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the simple_hri services with the chosen backend and language.

  ros2 launch simple_hri simple_hri.launch.py                      # local, Spanish (default)
  ros2 launch simple_hri simple_hri.launch.py language:=en
  ros2 launch simple_hri simple_hri.launch.py backend:=ollama ollama_model:=gemma3:4b
  ros2 launch simple_hri simple_hri.launch.py backend:=cloud tts:=mms
  ros2 launch simple_hri simple_hri.launch.py stt:=faster_whisper tts:=piper
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Backend of each service for each value of "backend"
BACKENDS = {
    'local': {'stt': 'whisper', 'tts': 'mms', 'extract': 'local', 'yesno': 'local'},
    'ollama': {'stt': 'whisper', 'tts': 'mms', 'extract': 'ollama', 'yesno': 'ollama'},
    'hf': {'stt': 'whisper', 'tts': 'mms', 'extract': 'hf', 'yesno': 'hf'},
    'cloud': {'stt': 'openai', 'tts': 'google', 'extract': 'openai', 'yesno': 'openai'},
}

# Options accepted by the per-service arguments
OPTIONS = {
    'stt': ['whisper', 'faster_whisper', 'openai'],
    'tts': ['mms', 'piper', 'google'],
    'extract': ['local', 'ollama', 'hf', 'openai'],
    'yesno': ['local', 'keywords', 'ollama', 'hf', 'openai'],
}

# Supported languages: code of the MMS-TTS model of each one
LANGUAGES = {'es': 'spa', 'en': 'eng'}

ARGUMENTS = [
    ('backend', 'local', 'Set of backends: local, ollama, hf (Hugging Face, paid) or cloud '
     '(OpenAI + Google, paid)'),
    ('language', 'es', 'Language of the conversation: es or en'),
    ('stt', '', 'STT backend, overrides the one of "backend": whisper (local), faster_whisper '
     '(local) or openai'),
    ('tts', '', 'TTS backend, overrides the one of "backend": mms (local), piper (local) or '
     'google'),
    ('extract', '', 'Extract backend, overrides the one of "backend": local, ollama, hf, openai'),
    ('yesno', '', 'Yes/No backend, overrides the one of "backend": local, keywords, ollama, hf, '
     'openai'),
    ('run_interaction_services', 'true',
     'true: STT, TTS, Extract and Yes/No. false: only the audio forwarding nodes'),
    ('tts_speaks', 'true', 'true: TTS plays the audio here. false: it sends it to '
     '/trigger_audio_send'),
    ('tts_lang', '', 'MMS-TTS model (facebook/mms-tts-<tts_lang>: spa, eng, fra...). '
     'Empty = from language'),
    ('audio_player', '', 'Command used to play audio (e.g. "aplay -q", "pw-play"). Empty = auto'),
    ('prompt_file', '', 'Prompts of Extract and Yes/No: a file in params/ or an absolute path. '
     'Empty = prompts_<language>.yaml'),
    ('whisper_model', 'tiny', 'Local Whisper model: tiny, base, small, medium, large '
     '(also of faster_whisper, which accepts large-v3, distil-large-v3...)'),
    ('piper_voice', '', 'Piper voice (a file name in models/ or an absolute path to its .onnx). '
     'Empty = es_ES-sharvard-medium or en_US-lessac-medium, from language'),
    ('piper_speaker', '-1', 'Speaker of a multi-speaker Piper voice (e.g. 0 or 1 of '
     "es_ES-sharvard-medium). -1 = the voice's default"),
    ('local_model', 'Qwen/Qwen2.5-1.5B-Instruct', 'Hugging Face model of the local LLM'),
    ('use_gpu', 'true', 'Run MMS-TTS and the local LLM on the GPU (CUDA), if there is one. '
     'Whisper uses it whenever it is available'),
    ('ollama_host', '', 'Ollama server (e.g. "localhost:11434", "myserver"). '
     'Empty = OLLAMA_HOST or localhost'),
    ('ollama_model', 'qwen2.5:3b', 'Ollama model for Extract and Yes/No'),
    ('ollama_keep_alive', '', 'How long Ollama keeps the model loaded (e.g. "1h", "-1" = '
     'forever). Empty = server default'),
    ('hf_model', 'meta-llama/Meta-Llama-3-8B-Instruct',
     'Hugging Face Inference Providers model for Extract and Yes/No'),
    ('openai_model', 'gpt-4.1-mini', 'OpenAI model for Extract and Yes/No'),
]


def to_bool(value):
    return value.lower() in ('true', '1', 'yes')


def node(executable, name, params):
    return Node(package='simple_hri', executable=executable, name=name, output='screen',
                parameters=[params])


def launch_nodes(context):
    arg = {name: LaunchConfiguration(name).perform(context) for name, _, _ in ARGUMENTS}

    audio_player = arg['audio_player']
    if not to_bool(arg['run_interaction_services']):
        return [
            node('audio_service', 'audio_service_node', {}),
            node('audio_file_player', 'audio_file_player_node', {'audio_player': audio_player}),
        ]

    backend = arg['backend']
    if backend not in BACKENDS:
        raise RuntimeError(f"Unknown backend '{backend}'. Options: {', '.join(BACKENDS)}")
    language = arg['language']
    if language not in LANGUAGES:
        raise RuntimeError(f"Unknown language '{language}'. Options: {', '.join(LANGUAGES)}")
    chosen = {}
    for service, default in BACKENDS[backend].items():
        chosen[service] = arg[service] or default
        if chosen[service] not in OPTIONS[service]:
            raise RuntimeError(f"Unknown {service} backend '{chosen[service]}'. "
                               f"Options: {', '.join(OPTIONS[service])}")

    use_gpu = to_bool(arg['use_gpu'])
    play_sound = to_bool(arg['tts_speaks'])
    prompt_file = arg['prompt_file'] or f'prompts_{language}.yaml'
    llm = {'prompt_file': prompt_file}
    ollama = {**llm, 'host': arg['ollama_host'], 'model': arg['ollama_model'],
              'keep_alive': arg['ollama_keep_alive']}

    # Parameters of each node
    params = {
        'stt_whisper': {'model': arg['whisper_model'], 'language': language},
        'stt_faster_whisper': {'model': arg['whisper_model'], 'language': language,
                               'device': 'auto' if use_gpu else 'cpu'},
        'stt_openai': {'language': language},
        'tts_mms': {'lang_code': arg['tts_lang'] or LANGUAGES[language], 'use_gpu': use_gpu,
                    'play_sound': play_sound, 'audio_player': audio_player},
        'tts_piper': {'lang_code': language, 'voice': arg['piper_voice'],
                      'speaker': int(arg['piper_speaker']),
                      'play_sound': play_sound, 'audio_player': audio_player},
        'tts_google': {'language': language, 'play_sound': play_sound,
                       'audio_player': audio_player},
        'extract_ollama': ollama,
        'extract_hf': {**llm, 'model_id': arg['hf_model']},
        'extract_openai': {**llm, 'model': arg['openai_model']},
        'yesno_keywords': {'language': language},
        'yesno_ollama': ollama,
        'yesno_hf': {**llm, 'model_id': arg['hf_model']},
        'yesno_openai': {**llm, 'model': arg['openai_model']},
    }

    print(f'simple_hri ({language}): ' + ', '.join(f'{s}={b}' for s, b in chosen.items()))

    nodes = []
    for service in ('stt', 'tts'):
        executable = f'{service}_{chosen[service]}'
        nodes.append(node(executable, f'{service}_service_node', params[executable]))

    # The local LLM is one node for Extract and Yes/No, so that the model is loaded once
    local = [s for s in ('extract', 'yesno') if chosen[s] == 'local']
    if local:
        nodes.append(node('llm_local', 'llm_local_node', {
            **llm, 'model_id': arg['local_model'], 'use_gpu': use_gpu,
            'extract': 'extract' in local, 'yesno': 'yesno' in local}))
    for service in ('extract', 'yesno'):
        if chosen[service] != 'local':
            executable = f'{service}_{chosen[service]}'
            nodes.append(node(executable, f'{service}_service_node', params[executable]))
    return nodes


def generate_launch_description():
    ld = LaunchDescription()
    for name, default, description in ARGUMENTS:
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))
    ld.add_action(OpaqueFunction(function=launch_nodes))
    return ld
