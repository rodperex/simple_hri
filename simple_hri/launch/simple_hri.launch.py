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

"""
Launch simple_hri: listen (STT), speak (TTS), extract information and detect yes/no.

HOW TO USE IT

  ros2 launch simple_hri simple_hri.launch.py                    # everything local, Spanish
  ros2 launch simple_hri simple_hri.launch.py language:=en       # in English
  ros2 launch simple_hri simple_hri.launch.py backend:=ollama    # LLM in Ollama (ollama serve)
  ros2 launch simple_hri simple_hri.launch.py backend:=ollama stt:=faster_whisper
  ros2 launch simple_hri simple_hri.launch.py llm_device:=cpu    # computer without a GPU
  ros2 launch simple_hri simple_hri.launch.py --show-args        # every argument

When it starts, it prints which node, model and device each capability uses. More cases
(no GPU, a lab server, another voice, speakers on another computer...): the section "What to
launch" of the README.

TWO LEVELS: backend AND ONE ENGINE PER CAPABILITY

Each capability can be done by several engines (Whisper or OpenAI to listen, Piper or
Google to speak...). "backend" chooses the engines of the four capabilities at once:

  backend   stt (listen)   tts (speak)   extract and yesno
  local     whisper        piper         local: Qwen2.5-1.5B on this computer (default)
  ollama    whisper        piper         ollama: ollama_model, served by Ollama
  hf        whisper        piper         hf: Hugging Face API (paid)
  cloud     openai         google        openai: OpenAI API (paid)

The arguments stt, tts, extract and yesno change the engine of ONE capability. Their
default, "backend", means "the one of the table above". Engines of each capability:

  stt      whisper, faster_whisper (both on this computer), openai
  tts      piper, mms (both on this computer), google
  extract  local, ollama, hf, openai
  yesno    local, keywords (a list of words, without an LLM), ollama, hf, openai

WHERE THE LOCAL MODELS RUN

stt_device, tts_device and llm_device: cpu, or gpu (the NVIDIA GPU if there is one; if
not, the CPU). By default the STT runs on the CPU and leaves the GPU to the LLM: a 4 GB GPU
does not fit Whisper small and the LLM together. Piper always runs on the CPU, and Ollama
decides by itself where its model runs.

HOW THIS FILE IS ORGANISED

  1. ARGUMENTS: every launch argument, its default and what it does
  2. BACKENDS and ENGINES: the tables above
     Helpers: small functions used by parts 3 and 4 (where, is_true, auto, check, node)
  3. stt_node(), tts_node() and llm_nodes(): which node each engine launches, and with
     which parameters
  4. generate_launch_description() and launch_nodes(): the main functions. They declare
     the arguments, choose the engine of each capability, create the nodes and print
     the summary
"""

from collections import namedtuple

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ============ 1. ARGUMENTS: (name, default, description) ============
# "backend" and "auto" are values with a meaning, explained in each description

ARGUMENTS = [
    # Which engines
    ('backend', 'local',
     'Set of engines for the four capabilities: local, ollama, hf (Hugging Face, paid) or '
     'cloud (OpenAI + Google, paid)'),
    ('language', 'es', 'Language of the conversation: es or en'),
    ('stt', 'backend',
     'Engine to listen: whisper, faster_whisper or openai. backend = the one of "backend"'),
    ('tts', 'backend',
     'Engine to speak: piper, mms or google. backend = the one of "backend"'),
    ('extract', 'backend',
     'Engine to extract information: local, ollama, hf or openai. '
     'backend = the one of "backend"'),
    ('yesno', 'backend',
     'Engine to detect yes/no: local, keywords, ollama, hf or openai. '
     'backend = the one of "backend"'),

    # Models
    # small: tiny garbles Spanish phrases from a laptop microphone
    ('whisper_model', 'small',
     'Whisper model of whisper and faster_whisper: tiny, base, small, medium, large... '
     '(bigger is better and slower)'),
    ('piper_voice', 'auto',
     'Piper voice: a file name in models/ or the path to its .onnx. '
     'auto = es_ES-sharvard-medium or en_US-lessac-medium, from language'),
    ('piper_speaker', '-1',
     'Speaker of a Piper voice with several (e.g. 0 or 1 of es_ES-sharvard-medium). '
     "-1 = the voice's default"),
    ('local_model', 'Qwen/Qwen2.5-1.5B-Instruct', 'Hugging Face model of the local LLM'),
    # 7b: the smaller ones mix up the values of several categories in one sentence
    ('ollama_model', 'qwen2.5:7b', 'Ollama model for extract and yesno'),
    ('hf_model', 'meta-llama/Meta-Llama-3-8B-Instruct',
     'Hugging Face model for extract and yesno'),
    ('openai_model', 'gpt-4.1-mini', 'OpenAI model for extract and yesno'),
    ('prompt_file', 'auto',
     'Prompts of extract and yesno: a file in params/ or an absolute path. '
     'auto = prompts_<language>.yaml'),

    # Where the local models run
    ('stt_device', 'cpu',
     'Where Whisper runs: cpu or gpu. On the CPU, small takes about 2 s per phrase'),
    ('tts_device', 'gpu', 'Where MMS-TTS runs (tts:=mms): cpu or gpu. Piper always uses the CPU'),
    ('llm_device', 'gpu',
     'Where the local LLM runs (extract or yesno local): cpu or gpu. '
     'On the CPU it takes several seconds per request'),

    # Audio and servers
    ('tts_speaks', 'true',
     'true: the robot speaks through the speakers of this computer. false: the audio is '
     'sent to another computer, which plays it (launched there with '
     'run_interaction_services:=false)'),
    ('audio_player', 'auto',
     'Command that plays the audio (e.g. "aplay -q", "pw-play"). auto = the one available'),
    ('run_interaction_services', 'true',
     'true: the four capabilities. false: only the node that plays the audio sent by the '
     'computer launched with tts_speaks:=false'),
    ('ollama_host', 'auto',
     'Ollama server (e.g. "localhost:11434", "myserver"). '
     'auto = the OLLAMA_HOST variable, or localhost'),
    ('ollama_keep_alive', 'auto',
     'How long Ollama keeps the model loaded (e.g. "1h", "-1" = forever). '
     'auto = the server default (5 minutes)'),
]

# ============ 2. BACKENDS and ENGINES ============

# Engine of each capability for each value of "backend". Piper: more natural voices than
# MMS-TTS, and fast on the CPU
BACKENDS = {
    'local': {'stt': 'whisper', 'tts': 'piper', 'extract': 'local', 'yesno': 'local'},
    'ollama': {'stt': 'whisper', 'tts': 'piper', 'extract': 'ollama', 'yesno': 'ollama'},
    'hf': {'stt': 'whisper', 'tts': 'piper', 'extract': 'hf', 'yesno': 'hf'},
    'cloud': {'stt': 'openai', 'tts': 'google', 'extract': 'openai', 'yesno': 'openai'},
}

# Engines accepted by the arguments stt, tts, extract and yesno
ENGINES = {
    'stt': ['whisper', 'faster_whisper', 'openai'],
    'tts': ['piper', 'mms', 'google'],
    'extract': ['local', 'ollama', 'hf', 'openai'],
    'yesno': ['local', 'keywords', 'ollama', 'hf', 'openai'],
}

# Supported languages, and the MMS-TTS model of each one (facebook/mms-tts-<code>)
LANGUAGES = {'es': 'spa', 'en': 'eng'}

DEVICES = ('cpu', 'gpu')

# The functions of part 3 return, for each capability, the node to launch:
#   executable  program of the simple_hri package, e.g. 'stt_whisper'
#   params      its ROS parameters, e.g. {'model': 'small', 'language': 'es'}
#   model       model shown in the summary when the launch starts, e.g. 'Whisper small'
#   device      where it runs, also for the summary, e.g. 'CPU'
# launch_nodes() creates a ROS Node from executable and params, and prints model and device.
# A namedtuple is a tuple whose fields are read by name: node.executable, node.params...
NodeToLaunch = namedtuple('NodeToLaunch', ['executable', 'params', 'model', 'device'])


# ---------- Helpers: small functions used by parts 3 and 4 ----------

def where(on_gpu):
    """Device shown in the summary for a local model."""
    return 'GPU if there is one' if on_gpu else 'CPU'


def is_true(value):
    """Value of a true/false argument."""
    return value.lower() in ('true', '1', 'yes')


def auto(value):
    """'auto' in an argument is an empty parameter for the nodes, which then decide."""
    return '' if value == 'auto' else value


def check(name, value, options):
    """Stop the launch with a clear message if an argument has an unknown value."""
    if value not in options:
        raise RuntimeError(f"Unknown {name} '{value}'. Options: {', '.join(options)}")


def node(executable, name, params):
    """ROS node of the simple_hri package, with its output on the screen."""
    return Node(package='simple_hri', executable=executable, name=name, output='screen',
                parameters=[params])


# ============ 3. WHICH NODE EACH ENGINE LAUNCHES ============
# A catalogue: one function per capability. Each one receives the engine chosen for that
# capability (e.g. stt = 'whisper') and returns the NodeToLaunch: which program of the
# simple_hri package runs it and with which parameters, built from the launch arguments.
#   stt_node()   listen  (/stt_action):    whisper, faster_whisper, openai
#   tts_node()   speak   (/tts_action):    piper, mms, google
#   llm_nodes()  extract (/extract_service) and yesno (/yesno_service), together because
#                with the engine local both share one node (the LLM is loaded once)
# To add an engine: add it to ENGINES (part 2) and add its case to the function here.

def stt_node(engine, arg, on_gpu):
    """Node that listens (/stt_action) with the chosen engine."""
    language = arg['language']
    device = 'auto' if on_gpu else 'cpu'  # 'auto' in the STT nodes: the GPU if there is one
    if engine == 'whisper':
        return NodeToLaunch(
            executable='stt_whisper',
            params={'model': arg['whisper_model'], 'language': language, 'device': device},
            model=f'Whisper {arg["whisper_model"]}',
            device=where(on_gpu))
    if engine == 'faster_whisper':
        return NodeToLaunch(
            executable='stt_faster_whisper',
            params={'model': arg['whisper_model'], 'language': language, 'device': device},
            model=f'faster-whisper {arg["whisper_model"]}',
            device=where(on_gpu))
    # openai
    return NodeToLaunch(
        executable='stt_openai',
        params={'language': language},
        model='whisper-1',
        device='OpenAI')


def tts_node(engine, arg, on_gpu):
    """Node that speaks (/tts_action) with the chosen engine."""
    language = arg['language']
    audio = {'play_sound': is_true(arg['tts_speaks']), 'audio_player': auto(arg['audio_player'])}
    if engine == 'piper':
        voice = auto(arg['piper_voice'])
        return NodeToLaunch(
            executable='tts_piper',
            params={'lang_code': language, 'voice': voice,
                    'speaker': int(arg['piper_speaker']), **audio},
            model=voice or 'Piper (voice of the language)',
            device='CPU')
    if engine == 'mms':
        mms_code = LANGUAGES[language]
        return NodeToLaunch(
            executable='tts_mms',
            params={'lang_code': mms_code, 'use_gpu': on_gpu, **audio},
            model=f'facebook/mms-tts-{mms_code}',
            device=where(on_gpu))
    # google
    return NodeToLaunch(
        executable='tts_google',
        params={'language': language, **audio},
        model='Google TTS',
        device='Google')


def llm_nodes(chosen, arg, on_gpu):
    """
    Nodes of extract and yesno: {capability: NodeToLaunch}.

    With the engine local, a single llm_local node serves both, so that the model is
    loaded only once: then both capabilities get the same NodeToLaunch.
    """
    prompts = {'prompt_file': auto(arg['prompt_file']) or f'prompts_{arg["language"]}.yaml'}
    nodes = {}
    local = [capability for capability in ('extract', 'yesno') if chosen[capability] == 'local']
    if local:
        shared = NodeToLaunch(
            executable='llm_local',
            params={**prompts, 'model_id': arg['local_model'], 'use_gpu': on_gpu,
                    'extract': 'extract' in local, 'yesno': 'yesno' in local},
            model=arg['local_model'],
            device=where(on_gpu))
        for capability in local:
            nodes[capability] = shared

    for capability in ('extract', 'yesno'):
        engine = chosen[capability]
        if engine == 'ollama':
            nodes[capability] = NodeToLaunch(
                executable=f'{capability}_ollama',
                params={**prompts, 'host': auto(arg['ollama_host']),
                        'model': arg['ollama_model'],
                        'keep_alive': auto(arg['ollama_keep_alive'])},
                model=arg['ollama_model'],
                device='Ollama (ollama ps: CPU or GPU)')
        elif engine == 'hf':
            nodes[capability] = NodeToLaunch(
                executable=f'{capability}_hf',
                params={**prompts, 'model_id': arg['hf_model']},
                model=arg['hf_model'],
                device='Hugging Face')
        elif engine == 'openai':
            nodes[capability] = NodeToLaunch(
                executable=f'{capability}_openai',
                params={**prompts, 'model': arg['openai_model']},
                model=arg['openai_model'],
                device='OpenAI')
        elif engine == 'keywords':  # only yesno
            nodes[capability] = NodeToLaunch(
                executable='yesno_keywords',
                params={'language': arg['language']},
                model='list of words',
                device='CPU')
    return nodes


# ============ 4. LAUNCH ============
# The two main functions, in the order they run:
#   generate_launch_description()  at the end of the file, as usual, but ROS 2 calls it
#       first (every launch file has one). It declares the arguments of part 1 and leaves
#       launch_nodes() ready to run
#   launch_nodes()  runs when the launch starts, once the argument values are known:
#       chooses the engine of each capability, gets its node from part 3, creates the
#       ROS nodes and prints the summary
# launch_nodes() needs the values of the arguments to decide which nodes to create, and they
# are only known when the launch starts: that is why it is wrapped in an OpaqueFunction.

def launch_nodes(context):
    """Read the arguments and return the nodes to launch (runs when the launch starts)."""
    arg = {name: LaunchConfiguration(name).perform(context) for name, _, _ in ARGUMENTS}

    # Computer with the speakers: only the node that plays the audio sent by the computer
    # that runs simple_hri with tts_speaks:=false
    if not is_true(arg['run_interaction_services']):
        return [node('audio_file_player', 'audio_file_player_node',
                     {'audio_player': auto(arg['audio_player'])})]

    check('backend', arg['backend'], BACKENDS)
    check('language', arg['language'], LANGUAGES)

    # Engine of each capability: its own argument, or the one of "backend"
    chosen = {}
    for capability in ('stt', 'tts', 'extract', 'yesno'):
        engine = arg[capability]
        if engine in ('backend', ''):
            engine = BACKENDS[arg['backend']][capability]
        check(capability, engine, ENGINES[capability])
        chosen[capability] = engine

    # Where each local model runs
    on_gpu = {}
    for part in ('stt', 'tts', 'llm'):
        check(f'{part}_device', arg[f'{part}_device'], DEVICES)
        on_gpu[part] = arg[f'{part}_device'] == 'gpu'

    # The node to launch for each capability (part 3)
    to_launch = {
        'stt': stt_node(chosen['stt'], arg, on_gpu['stt']),
        'tts': tts_node(chosen['tts'], arg, on_gpu['tts']),
        **llm_nodes(chosen, arg, on_gpu['llm']),
    }

    # Create the ROS nodes. llm_local appears twice (extract and yesno) but is launched once
    nodes = []
    launched = []
    for capability, node_to_launch in to_launch.items():
        if node_to_launch in launched:
            continue
        launched.append(node_to_launch)
        if node_to_launch.executable == 'llm_local':
            name = 'llm_local_node'
        else:
            name = f'{capability}_service_node'
        nodes.append(node(node_to_launch.executable, name, node_to_launch.params))

    # tts_speaks:=false: the TTS writes the audio to a WAV file and calls /trigger_audio_send.
    # audio_service, on this same computer because it reads the file, publishes it for the
    # audio_file_player of the computer with the speakers
    if not is_true(arg['tts_speaks']):
        nodes.append(node('audio_service', 'audio_service_node', {}))

    # Summary: which node, model and device each capability uses
    print(f'simple_hri ({arg["language"]}, backend {arg["backend"]}):')
    for capability, node_to_launch in to_launch.items():
        print(f'  {capability:8s} {node_to_launch.executable:19s} '
              f'{node_to_launch.model:36s} {node_to_launch.device}')
    return nodes


def generate_launch_description():
    """Declare the arguments and, once their values are known, launch the nodes."""
    ld = LaunchDescription()
    for name, default, description in ARGUMENTS:
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))
    ld.add_action(OpaqueFunction(function=launch_nodes))
    return ld
