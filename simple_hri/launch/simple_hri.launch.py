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

"""Launch the simple_hri services: STT, TTS, Extract and Yes/No.

Each service has a backend. "backend" chooses all of them at once:

  backend   STT                TTS             Extract and Yes/No
  local     Whisper            MMS-TTS         Qwen2.5-1.5B (transformers)
  ollama    Whisper            MMS-TTS         ollama_model, served by Ollama
  hf        Whisper            MMS-TTS         hf_model, Hugging Face API (paid)
  cloud     OpenAI whisper-1   Google TTS      openai_model, OpenAI API (paid)

and stt, tts, extract and yesno change the backend of one service.

The local models run where *_device says: cpu, or gpu (the NVIDIA GPU if there is one,
otherwise the CPU). By default the STT runs on the CPU, so that the GPU is left to
MMS-TTS and the LLM: a 4 GB GPU does not fit the three. Piper always runs on the CPU, and
Ollama decides by itself where its model runs. When the launch starts, it prints which
node, model and device each service uses.

  ros2 launch simple_hri simple_hri.launch.py                      # local, Spanish
  ros2 launch simple_hri simple_hri.launch.py language:=en
  ros2 launch simple_hri simple_hri.launch.py stt:=faster_whisper tts:=piper
  ros2 launch simple_hri simple_hri.launch.py backend:=ollama      # needs ollama serve
  ros2 launch simple_hri simple_hri.launch.py llm_device:=cpu      # no GPU at all
  ros2 launch simple_hri simple_hri.launch.py backend:=cloud tts:=mms
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

DEVICES = ('cpu', 'gpu')

ARGUMENTS = [
    # 1. Backend of each service
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

    # 2. Models
    # small: tiny garbles Spanish phrases from a laptop microphone
    ('whisper_model', 'small', 'Local Whisper model: tiny, base, small, medium, large '
     '(also of faster_whisper, which accepts large-v3, distil-large-v3...)'),
    ('tts_lang', '', 'MMS-TTS model (facebook/mms-tts-<tts_lang>: spa, eng, fra...). '
     'Empty = from language'),
    ('piper_voice', '', 'Piper voice (a file name in models/ or an absolute path to its .onnx). '
     'Empty = es_ES-sharvard-medium or en_US-lessac-medium, from language'),
    ('piper_speaker', '-1', 'Speaker of a multi-speaker Piper voice (e.g. 0 or 1 of '
     "es_ES-sharvard-medium). -1 = the voice's default"),
    ('local_model', 'Qwen/Qwen2.5-1.5B-Instruct', 'Hugging Face model of the local LLM'),
    # 7b: the smaller ones mix up the values of several categories in one sentence
    ('ollama_model', 'qwen2.5:7b', 'Ollama model for Extract and Yes/No'),
    ('hf_model', 'meta-llama/Meta-Llama-3-8B-Instruct',
     'Hugging Face Inference Providers model for Extract and Yes/No'),
    ('openai_model', 'gpt-4.1-mini', 'OpenAI model for Extract and Yes/No'),
    ('prompt_file', '', 'Prompts of Extract and Yes/No: a file in params/ or an absolute path. '
     'Empty = prompts_<language>.yaml'),

    # 3. Where the local models run: cpu, or gpu (the NVIDIA GPU if there is one, else the CPU)
    ('stt_device', 'cpu', 'Device of Whisper and faster-whisper: cpu or gpu. On the CPU, small '
     'takes about 2 s per phrase'),
    ('tts_device', 'gpu', 'Device of MMS-TTS: cpu or gpu (Piper always runs on the CPU)'),
    ('llm_device', 'gpu', 'Device of the local LLM (backend local): cpu or gpu. On the CPU it '
     'takes several seconds per request'),
    ('use_gpu', 'true', 'false: every local model on the CPU, whatever *_device says'),

    # 4. Audio and servers
    ('tts_speaks', 'true', 'true: TTS plays the audio here. false: it sends it to '
     '/trigger_audio_send'),
    ('audio_player', '', 'Command used to play audio (e.g. "aplay -q", "pw-play"). Empty = auto'),
    ('run_interaction_services', 'true',
     'true: STT, TTS, Extract and Yes/No. false: only the audio forwarding nodes'),
    ('ollama_host', '', 'Ollama server (e.g. "localhost:11434", "myserver"). '
     'Empty = OLLAMA_HOST or localhost'),
    ('ollama_keep_alive', '', 'How long Ollama keeps the model loaded (e.g. "1h", "-1" = '
     'forever). Empty = server default'),
]


def to_bool(value):
    return value.lower() in ('true', '1', 'yes')


def node(executable, name, params):
    return Node(package='simple_hri', executable=executable, name=name, output='screen',
                parameters=[params])


def check(name, value, options):
    if value not in options:
        raise RuntimeError(f"Unknown {name} '{value}'. Options: {', '.join(options)}")


def launch_nodes(context):
    arg = {name: LaunchConfiguration(name).perform(context) for name, _, _ in ARGUMENTS}

    audio_player = arg['audio_player']
    if not to_bool(arg['run_interaction_services']):
        return [
            node('audio_service', 'audio_service_node', {}),
            node('audio_file_player', 'audio_file_player_node', {'audio_player': audio_player}),
        ]

    # Backend of each service: the one of "backend", unless its own argument says another
    check('backend', arg['backend'], BACKENDS)
    language = arg['language']
    check('language', language, LANGUAGES)
    chosen = {}
    for service, default in BACKENDS[arg['backend']].items():
        chosen[service] = arg[service] or default
        check(f'{service} backend', chosen[service], OPTIONS[service])

    # Device of each local model. 'gpu' means "the GPU if there is one": the nodes fall back
    # to the CPU by themselves
    gpu_allowed = to_bool(arg['use_gpu'])
    on_gpu = {}
    for part in ('stt', 'tts', 'llm'):
        check(f'{part}_device', arg[f'{part}_device'], DEVICES)
        on_gpu[part] = gpu_allowed and arg[f'{part}_device'] == 'gpu'
    stt_device = 'auto' if on_gpu['stt'] else 'cpu'  # Values of the STT nodes

    play_sound = to_bool(arg['tts_speaks'])
    tts_lang = arg['tts_lang'] or LANGUAGES[language]
    llm = {'prompt_file': arg['prompt_file'] or f'prompts_{language}.yaml'}
    ollama = {**llm, 'host': arg['ollama_host'], 'model': arg['ollama_model'],
              'keep_alive': arg['ollama_keep_alive']}

    # Node of each backend: (executable, parameters, model and device shown in the summary)
    where = {True: 'GPU if there is one', False: 'CPU'}
    backends = {
        ('stt', 'whisper'): ('stt_whisper', {
            'model': arg['whisper_model'], 'language': language, 'device': stt_device},
            f'Whisper {arg["whisper_model"]}', where[on_gpu['stt']]),
        ('stt', 'faster_whisper'): ('stt_faster_whisper', {
            'model': arg['whisper_model'], 'language': language, 'device': stt_device},
            f'faster-whisper {arg["whisper_model"]}', where[on_gpu['stt']]),
        ('stt', 'openai'): ('stt_openai', {'language': language}, 'whisper-1', 'OpenAI'),
        ('tts', 'mms'): ('tts_mms', {
            'lang_code': tts_lang, 'use_gpu': on_gpu['tts'],
            'play_sound': play_sound, 'audio_player': audio_player},
            f'facebook/mms-tts-{tts_lang}', where[on_gpu['tts']]),
        ('tts', 'piper'): ('tts_piper', {
            'lang_code': language, 'voice': arg['piper_voice'],
            'speaker': int(arg['piper_speaker']),
            'play_sound': play_sound, 'audio_player': audio_player},
            arg['piper_voice'] or 'Piper (voice of the language)', 'CPU'),
        ('tts', 'google'): ('tts_google', {
            'language': language, 'play_sound': play_sound, 'audio_player': audio_player},
            'Google TTS', 'Google'),
        ('extract', 'ollama'): ('extract_ollama', ollama, arg['ollama_model'], 'Ollama'),
        ('extract', 'hf'): ('extract_hf', {**llm, 'model_id': arg['hf_model']},
                            arg['hf_model'], 'Hugging Face'),
        ('extract', 'openai'): ('extract_openai', {**llm, 'model': arg['openai_model']},
                                arg['openai_model'], 'OpenAI'),
        ('yesno', 'keywords'): ('yesno_keywords', {'language': language},
                                'list of words', 'CPU'),
        ('yesno', 'ollama'): ('yesno_ollama', ollama, arg['ollama_model'], 'Ollama'),
        ('yesno', 'hf'): ('yesno_hf', {**llm, 'model_id': arg['hf_model']},
                          arg['hf_model'], 'Hugging Face'),
        ('yesno', 'openai'): ('yesno_openai', {**llm, 'model': arg['openai_model']},
                              arg['openai_model'], 'OpenAI'),
    }

    nodes = []
    summary = []
    for service in ('stt', 'tts', 'extract', 'yesno'):
        if chosen[service] == 'local':
            continue  # Below: one node for both services
        executable, params, model, device = backends[(service, chosen[service])]
        nodes.append(node(executable, f'{service}_service_node', params))
        summary.append((service, executable, model, device))

    # The local LLM is one node for Extract and Yes/No, so that the model is loaded once
    local = [s for s in ('extract', 'yesno') if chosen[s] == 'local']
    if local:
        nodes.append(node('llm_local', 'llm_local_node', {
            **llm, 'model_id': arg['local_model'], 'use_gpu': on_gpu['llm'],
            'extract': 'extract' in local, 'yesno': 'yesno' in local}))
        for service in local:
            summary.append((service, 'llm_local', arg['local_model'], where[on_gpu['llm']]))

    print(f'simple_hri ({language}):')
    for service, executable, model, device in summary:
        print(f'  {service:8s} {executable:19s} {model:36s} {device}')
    return nodes


def generate_launch_description():
    ld = LaunchDescription()
    for name, default, description in ARGUMENTS:
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))
    ld.add_action(OpaqueFunction(function=launch_nodes))
    return ld
