# simple_hri

A lightweight set of ROS 2 nodes for Human-Robot Interaction (HRI): speech-to-text (STT),
text-to-speech (TTS), information extraction with an LLM and yes/no detection.

Listening and speaking last seconds, so they are **actions** (feedback, cancellation, and
speaking finishes when the audio ends); extracting information and yes/no detection are
queries on a text, so they are **services**. Each capability can be done by several
**engines** (local models, Ollama, Hugging Face, cloud APIs), all with the same ROS 2
interface, so a client works with any of them. A single launch file starts them: a
**backend** chooses a set of engines at once, and each capability can also change its own.

## Features

- **STT** (action): listens to the microphone, detects voice (VAD), records until silence and
  transcribes it. Engines: local Whisper, local faster-whisper (with Silero VAD) or the
  OpenAI API.
- **TTS** (action): synthesizes a text and plays it (or sends it to another device). Engines: local
  `facebook/mms-tts-*` models, local Piper voices or Google Cloud.
- **Extract** (service): given a text and an *interest* (e.g. "bebida"), returns only the relevant
  words. Engines: local LLM, Ollama, Hugging Face or OpenAI.
- **Yes/No** (service): tells whether an answer is affirmative, negative or neither. Engines: local
  LLM, word lists, Ollama, Hugging Face or OpenAI.

## Requirements

- **ROS 2 Jazzy** (or compatible versions)
- **Python 3.12+**
- A microphone and speakers, and an audio player for TTS: `aplay` (package `alsa-utils`),
  `pw-play` or `paplay`
- Optional: an NVIDIA GPU (4 GB are enough) for the local models, and about 4 GB of disk for
  them

## Installation

### 1. Clone the repository into your ROS 2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/rodperex/simple_hri.git
```

### 2. Install Python dependencies

```bash
cd ~/ros2_ws/src/simple_hri
pip install -r simple_hri/requirements.txt
```

### 3. Build the workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Quick start

1. Launch the services, fully local and free (Spanish):

   ```bash
   ros2 launch simple_hri simple_hri.launch.py
   ```

   The first time it downloads the models (about 3.5 GB), so it takes a while. It is ready when
   the log shows `Service "extract_service" ready`, `Service "yesno_service" ready` and the
   STT and TTS nodes are initialized.

2. In another terminal (same workspace and `ROS_DOMAIN_ID`), try it:

   ```bash
   ros2 action send_goal /tts_action simple_hri_interfaces/action/Say "{text: 'Hola, soy un robot'}"
   ros2 action send_goal /stt_action simple_hri_interfaces/action/Listen "{max_wait: 8.0}"   # then speak
   ros2 run simple_hri test_llm                                                # Extract and Yes/No
   ```

3. For other languages, computers or models, find your case in
   [What to launch](#what-to-launch), and use the actions and services from your nodes (see
   [Actions](#actions) and [Services](#services)).

## What to launch

Find your case and copy the command. The arguments can be combined, e.g.
`backend:=ollama language:=en stt:=faster_whisper`.

| I want... | Command (`ros2 launch simple_hri simple_hri.launch.py` + ...) |
| --- | --- |
| Everything on this computer, free, in Spanish | *(nothing: the default)* |
| The conversation in English | `language:=en` |
| A computer without an NVIDIA GPU | `backend:=ollama` (Ollama answers in about 2 s on the CPU), or `llm_device:=cpu yesno:=keywords` (no Ollama, but Extract takes 10-30 s) |
| Several things in one sentence (a drink, a dish and a dessert in one order) | `backend:=ollama` (`qwen2.5:7b` separates them; the local model mixes them up) |
| The LLM on a server of the lab | `backend:=ollama ollama_host:=server` (see [3b](#3b-use-ollama-on-a-server)) |
| Better listening in a noisy room | `stt:=faster_whisper` |
| Another voice | `piper_voice:=es_ES-davefx-medium`, or the other speaker of the default one: `piper_speaker:=1` |
| Yes/no without an LLM, instantly (simple answers only) | `yesno:=keywords` |
| A GPU with 8 GB or more: everything on the GPU, bigger models | `stt_device:=gpu whisper_model:=medium local_model:=Qwen/Qwen2.5-3B-Instruct` |
| Better transcriptions (paid), everything else local | `stt:=openai` (with `OPENAI_API_KEY`) |
| Everything in the cloud (paid) | `backend:=cloud` (with `OPENAI_API_KEY` and `GOOGLE_APPLICATION_CREDENTIALS`) |
| The models on this computer and the speakers on another one (e.g. the robot) | `tts_speaks:=false` here, and `run_interaction_services:=false` on the other one (see [Audio forwarding](#audio-forwarding)) |
| See every argument, without launching anything | `--show-args` |

When it starts, the launch prints which node, model and device each capability uses: check
there that it is what you wanted.

### What is running

Each line of that summary is one node (one program). With `backend:=ollama`:

```text
simple_hri (es, backend ollama):
  stt      stt_whisper         Whisper small                        CPU
  tts      tts_piper           Piper (voice of the language)        CPU
  extract  extract_ollama      qwen2.5:7b                           Ollama (ollama ps: CPU or GPU)
  yesno    yesno_ollama        qwen2.5:7b                           Ollama (ollama ps: CPU or GPU)
```

`extract_ollama` and `yesno_ollama` load no model: they send each sentence to the Ollama
server, which runs the model where it decides. To follow what is running:

| To see... | Run |
| --- | --- |
| Which nodes are running | `ros2 node list` (`/stt_service_node`, `/tts_service_node`, `/extract_service_node`, `/yesno_service_node`, or `/llm_local_node`) |
| What each node does | The terminal of the launch: every line starts with its node, e.g. `[stt_whisper-1]` |
| A parameter of a node | `ros2 param get /stt_service_node device` |
| Where the Ollama model runs | `ollama ps`, on the Ollama computer: `PROCESSOR` is `100% GPU`, `100% CPU` or both |
| What uses the GPU | `nvidia-smi`: with `backend:=local`, the process of `llm_local`; with `backend:=ollama`, only Ollama (if its model fits) |
| That the capabilities answer | `ros2 action list` and `ros2 service list` (see [Testing](#testing)) |

## Launching

```bash
ros2 launch simple_hri simple_hri.launch.py backend:=<backend> [other arguments]
```

It starts the two actions (`/stt_action`, `/tts_action`) and the two services
(`/extract_service`, `/yesno_service`). The interfaces are the same with every backend, so
your client does not change.

Each capability (listen, speak, extract, yes/no) can be done by several **engines**: Whisper
or OpenAI to listen, Piper or Google to speak... There are two levels of arguments:

- `backend` chooses a set of engines for the four capabilities at once (table below).
- `stt`, `tts`, `extract` and `yesno` change the engine of **one** capability. Their default,
  `backend`, means "the one of the set" (see [Mixing engines](#mixing-engines)).

### Backends

| `backend` | STT | TTS | Extract | Yes/No | Needs | Cost |
| --- | --- | --- | --- | --- | --- | --- |
| [`local`](#local) (default) | Whisper | Piper | Small LLM (transformers) | The same LLM | Nothing | Free |
| [`ollama`](#ollama) | Whisper | Piper | Ollama | Ollama | An Ollama server, on this computer or another one | Free |
| [`hf`](#hugging-face) | Whisper | Piper | Hugging Face API | Hugging Face API | `HF_TOKEN` | Paid: Hugging Face credits |
| [`cloud`](#cloud) | OpenAI API | Google Cloud | OpenAI API | OpenAI API | `OPENAI_API_KEY`, `GOOGLE_APPLICATION_CREDENTIALS` | Paid: OpenAI credits, Google Cloud billing |

Whisper and Piper always run on your computer: Ollama only serves language models, it
cannot transcribe or synthesize audio.

Two more local engines can be used with any backend: faster-whisper for STT
(`stt:=faster_whisper`) and MMS-TTS for TTS (`tts:=mms`, the default before Piper); see
[faster-whisper and Piper](#faster-whisper-and-piper).

Each capability can also use an engine different from the one of `backend`, with the
arguments `stt`, `tts`, `extract` and `yesno` (see [Mixing engines](#mixing-engines)).

The environment variables (`HF_TOKEN`, `OPENAI_API_KEY`...) must be exported **in the
terminal where you launch** (or in your `~/.bashrc`). The nodes read them at startup.

### Launch arguments

They are grouped as in the launch file: which engines, which models, where the local models
run, and audio and servers. Two values have a meaning of their own: `backend` (the engine of
the set chosen with `backend`) and `auto` (the nodes decide, as described).

**1. Which engines**

`backend` chooses a set of engines for the four capabilities at once; `stt`, `tts`, `extract`
and `yesno` change the engine of only one of them (see [Mixing engines](#mixing-engines)).

| Argument | Default | Description |
| --- | --- | --- |
| `backend` | `local` | `local`, `ollama`, `hf` or `cloud` (see [Backends](#backends)) |
| `language` | `es` | Language of the conversation: `es` (Spanish) or `en` (English). It sets the language of STT, the TTS voice, the prompts and the word lists |
| `stt` | `backend` | `whisper` (local), `faster_whisper` (local) or `openai`. `backend` = the one of `backend` |
| `tts` | `backend` | `piper` (local), `mms` (local) or `google`. `backend` = the one of `backend` |
| `extract` | `backend` | `local`, `ollama`, `hf` or `openai`. `backend` = the one of `backend` |
| `yesno` | `backend` | `local`, `keywords` (word lists, no model), `ollama`, `hf` or `openai`. `backend` = the one of `backend` |

**2. Models**

| Argument | Default | Description |
| --- | --- | --- |
| `whisper_model` | `small` | Local Whisper model: `tiny`, `base`, `small`, `medium`, `large`. Bigger is better and slower; `tiny` garbles Spanish phrases from a laptop microphone. faster-whisper uses it too (and accepts `large-v3`, `distil-large-v3`...) |
| `piper_voice` | `auto` | Piper voice: a voice of the [Piper catalog](https://huggingface.co/rhasspy/piper-voices) (`es_ES-davefx-medium`), a file name in `models/` or an absolute path to its `.onnx`. `auto` = `es_ES-sharvard-medium` or `en_US-lessac-medium`, from `language` |
| `piper_speaker` | `-1` | Speaker of a multi-speaker Piper voice (`es_ES-sharvard-medium` has 0 and 1). `-1` = the voice's default |
| `local_model` | `Qwen/Qwen2.5-1.5B-Instruct` | Hugging Face model of the local LLM (Extract and Yes/No) |
| `ollama_model` | `qwen2.5:7b` | Ollama model for Extract and Yes/No. It must be downloaded on the server (`ollama pull`). The smallest that separates several categories in one sentence (see [Choosing a model](#2-download-a-model)) |
| `hf_model` | `meta-llama/Meta-Llama-3-8B-Instruct` | Hugging Face model for Extract and Yes/No |
| `openai_model` | `gpt-4.1-mini` | OpenAI model for Extract and Yes/No |
| `prompt_file` | `auto` | Prompts of Extract and Yes/No: a file in `params/` or an absolute path (see [Prompts](#prompts)). `auto` = `prompts_<language>.yaml` |

**3. Where the local models run**

Each one is `cpu` or `gpu`; `gpu` means the NVIDIA GPU if there is one, otherwise the CPU.
By default the STT runs on the CPU so that the GPU is left to the local LLM (and to MMS-TTS,
if you choose it): a 4 GB GPU does not fit Whisper `small` and the LLM together. Piper always runs on the CPU, and Ollama decides by itself
where its model runs.

| Argument | Default | Description |
| --- | --- | --- |
| `stt_device` | `cpu` | Whisper and faster-whisper. On the CPU, `small` takes about 2 s per phrase |
| `tts_device` | `gpu` | MMS-TTS (`tts:=mms`). Piper always runs on the CPU |
| `llm_device` | `gpu` | The local LLM (`backend:=local`). On the CPU it takes several seconds per request |

**4. Audio and servers**

| Argument | Default | Description |
| --- | --- | --- |
| `background_noise` | `medium` | Noise of the room while listening (`stt` `whisper` and `openai`): `low`, `medium` or `high` (4, 6 or 8 dB that a voice needs above the background; the values are in `BACKGROUND_NOISE`, in the launch file), or a number of dB. Higher: less noise taken for voice, but the person must speak louder or closer (see [STT does not hear anything](#stt-does-not-hear-anything-no-se-detectó-audio-o-timeout)) |
| `tts_speaks` | `true` | `true`: TTS plays the audio on this computer. `false`: it sends the file to `/trigger_audio_send` |
| `audio_player` | `auto` | Command used to play audio (`aplay -q`, `pw-play`, `paplay`). `auto` = the first one installed |
| `run_interaction_services` | `true` | `true`: STT, TTS, Extract and Yes/No. `false`: only the audio forwarding nodes (see [Audio forwarding](#audio-forwarding)) |
| `ollama_host` | `auto` | Ollama server (`localhost:11434`, `server`, `http://server:11434`). `auto` = `OLLAMA_HOST` or `localhost:11434` |
| `ollama_keep_alive` | `auto` | How long Ollama keeps the model loaded (`1h`, `-1` = forever). `auto` = server default (5 minutes) |

`ros2 launch simple_hri simple_hri.launch.py --show-args` lists them too. When it starts,
the launch prints the node, the model and the device of each capability:

```text
simple_hri (es, backend local):
  stt      stt_whisper         Whisper small                        CPU
  tts      tts_piper           Piper (voice of the language)        CPU
  extract  llm_local           Qwen/Qwen2.5-1.5B-Instruct           GPU if there is one
  yesno    llm_local           Qwen/Qwen2.5-1.5B-Instruct           GPU if there is one
```

The local models (Whisper, faster-whisper, MMS-TTS, Piper voices, the local LLM) are
downloaded the first time to a `models/` directory **in the current working directory**:
launch always from the same directory, or they are downloaded again. Silero VAD (used by
faster-whisper) is downloaded by `torch.hub` to `~/.cache/torch/hub`.

### Language

`language:=es` (default) or `language:=en` sets the language of the whole conversation: what
STT transcribes, the TTS voice, the prompts of the LLMs (`params/prompts_<language>.yaml`)
and the word lists of `yesno_keywords`.

```bash
ros2 launch simple_hri simple_hri.launch.py language:=en
ros2 launch simple_hri simple_hri.launch.py backend:=ollama language:=en
```

### Local

Everything runs on your computer, with no account and no cost.

```bash
ros2 launch simple_hri simple_hri.launch.py
```

| Capability | Node | Model |
| --- | --- | --- |
| STT | `stt_whisper` | Whisper `small`, on the CPU (`stt_device`) |
| TTS | `tts_piper` | Piper voice `es_ES-sharvard-medium` (`en_US-lessac-medium` with `language:=en`), on the CPU |
| Extract | `llm_local` | `Qwen/Qwen2.5-1.5B-Instruct`, on the GPU if there is one (`llm_device`) |
| Yes/No | `llm_local` | The same model: `llm_local` serves both services, so the model is loaded once |

**Needs:** nothing, but an NVIDIA GPU makes Extract and Yes/No much faster. The first launch
needs Internet to download about 3.5 GB of models.

**Speed.** Extract and Yes/No run a 1.5B LLM. On an NVIDIA GPU it takes about 3 GB and answers
in 0.15-0.3 s; on the CPU it takes 10-30 s per request. Without a GPU, use
`local_model:=Qwen/Qwen2.5-0.5B-Instruct` (a few seconds, but many more mistakes), Ollama on
another machine (`extract:=ollama yesno:=ollama ollama_host:=server`), or the word lists for
Yes/No (see below).

**Yes/No: model or word lists.** Yes/No can be answered in two ways:

| | Model (`yesno:=local`, default) | Word lists (`yesno:=keywords`) |
| --- | --- | --- |
| How | The LLM reads the answer and classifies it | Looks for known words in the answer |
| Understands | Free-form answers, including doubt (`UNKNOWN`): "ahora no, quizá luego", "no estoy seguro". It can still make mistakes (19/20 in [Testing](#testing)) | Only the words in its lists: sí, vale, claro, venga... / no, nunca, qué va... (and their English equivalents). An answer without them is `UNKNOWN` |
| Speed | 0.15 s on a GPU, 10-30 s on the CPU | Instant |
| Memory | None extra: it shares the model with Extract | None |
| Use it | By default | Without a GPU, when the answers are simple (sí/no) and speed matters |

Both are in the language of `language`. With the model, Extract and Yes/No run in the same
node (`llm_local`), which loads the model once: two copies would not fit in a 4 GB GPU.

```bash
ros2 launch simple_hri simple_hri.launch.py yesno:=keywords   # word lists instead of the model
```

Bigger models, and the STT on the GPU too, if your GPU has more memory (8 GB or more):

```bash
ros2 launch simple_hri simple_hri.launch.py stt_device:=gpu whisper_model:=medium local_model:=Qwen/Qwen2.5-3B-Instruct
```

### Ollama

Same STT and TTS as `local`, but Extract and Yes/No use an LLM served by Ollama, on your GPU
or on a server. Free, and the best quality without paying.

```bash
ros2 launch simple_hri simple_hri.launch.py backend:=ollama
```

| Capability | Node | Model |
| --- | --- | --- |
| STT | `stt_whisper` | Whisper `small` |
| TTS | `tts_piper` | Piper voice `es_ES-sharvard-medium` (`en_US-lessac-medium` with `language:=en`) |
| Extract | `extract_ollama` | `ollama_model` (`qwen2.5:7b`) |
| Yes/No | `yesno_ollama` | `ollama_model` (`qwen2.5:7b`) |

**Needs:** an Ollama server with the model downloaded. See
[Ollama: LLM on your GPU or on a server](#ollama-llm-on-your-gpu-or-on-a-server) to set it up.

```bash
# Ollama on this computer, a smaller model (faster, but it mixes up several categories)
ros2 launch simple_hri simple_hri.launch.py backend:=ollama ollama_model:=qwen2.5:3b
# Ollama on a server, through an SSH tunnel (localhost:11434 is the server)
ssh -N -L 11434:127.0.0.1:11434 user@server &
ros2 launch simple_hri simple_hri.launch.py backend:=ollama ollama_model:=qwen2.5:14b
# Ollama on a server with the port open, keeping the model loaded for one hour
ros2 launch simple_hri simple_hri.launch.py backend:=ollama ollama_host:=server ollama_model:=qwen2.5:14b ollama_keep_alive:=1h
```

### Hugging Face

Same STT and TTS as `local`, but Extract and Yes/No call an LLM of the Hugging Face
[Inference Providers](https://huggingface.co/docs/inference-providers) API
(`meta-llama/Meta-Llama-3-8B-Instruct` by default).

```bash
export HF_TOKEN="hf_..."
ros2 launch simple_hri simple_hri.launch.py backend:=hf
```

| Capability | Node | Model |
| --- | --- | --- |
| STT | `stt_whisper` | Whisper `small` |
| TTS | `tts_piper` | Piper voice `es_ES-sharvard-medium` (`en_US-lessac-medium` with `language:=en`) |
| Extract | `extract_hf` | `hf_model` (remote) |
| Yes/No | `yesno_hf` | `hf_model` (remote) |

**Needs:**

1. A Hugging Face account and an access token: <https://huggingface.co/settings/tokens>.
   A *fine-grained* token needs the permission "Make calls to Inference Providers".
2. **Credits.** This is not free: since 2025 free accounts get no monthly credits for
   Inference Providers. You need to buy credits
   (<https://huggingface.co/settings/billing>) or a PRO subscription ($2 of credits per
   month). Without credits the services answer `ERROR...` and the log shows
   `402 Payment Required`. See the [pricing](https://huggingface.co/docs/inference-providers/pricing).
3. Llama models are gated: accept the license on the
   [model page](https://huggingface.co/meta-llama/Meta-Llama-3-8B-Instruct) with your account.

The `ollama` backend gives the same (or better) results for free, if you have a GPU or
access to a server.

### Cloud

STT, Extract and Yes/No use the OpenAI API, and TTS uses Google Cloud. Best STT quality and
no local models, but every request costs money and needs Internet.

```bash
export OPENAI_API_KEY="sk-..."
export GOOGLE_APPLICATION_CREDENTIALS="/path/to/the/key.json"
ros2 launch simple_hri simple_hri.launch.py backend:=cloud
```

| Capability | Node | Model |
| --- | --- | --- |
| STT | `stt_openai` | OpenAI `whisper-1` |
| TTS | `tts_google` | Google Cloud TTS, voice `es-ES-Neural2-F` (`en-US-Neural2-D` with `language:=en`) |
| Extract | `extract_openai` | `openai_model` (`gpt-4.1-mini`) |
| Yes/No | `yesno_openai` | `openai_model` (`gpt-4.1-mini`) |

**Needs:**

1. **OpenAI API key with credits** (for STT, which uses the OpenAI Whisper API, Extract and
   Yes/No). Create the key in <https://platform.openai.com/api-keys> and add
   prepaid credits in <https://platform.openai.com/settings/organization/billing>. A ChatGPT
   subscription (Plus, Pro...) does **not** include API usage. Without credits the requests
   fail with `429 insufficient_quota`. Prices: <https://openai.com/api/pricing>.
2. **Google Cloud credentials** (only for TTS).
   1. Create a project in the [Google Cloud console](https://console.cloud.google.com) and
      link a billing account to it (required even for the free monthly quota; new accounts
      get trial credits).
   2. Enable the **Cloud Text-to-Speech API** in the project.
   3. Create a service account, and a JSON key for it (*IAM & Admin → Service accounts →
      Keys → Add key → JSON*). Keep the file out of any repository.

   Instructions: [Text-to-Speech quickstart](https://cloud.google.com/text-to-speech/docs/quickstart-client-libraries).
   Prices and free quota: <https://cloud.google.com/text-to-speech/pricing>.

If `OPENAI_API_KEY` is missing, the OpenAI nodes exit at startup; if the Google credentials
are missing, `tts_google` fails at startup.

### faster-whisper and Piper

Two local engines, from the speech stack of
[DialoStack](https://github.com/aquintan4/DialoStack), with the same services and actions as
the others, so clients do not change. Piper is the default TTS of the `local`, `ollama` and
`hf` backends; faster-whisper is chosen with `stt:=faster_whisper`:

| | `stt:=faster_whisper` | `tts:=piper` (default) |
| --- | --- | --- |
| Node | `stt_faster_whisper` | `tts_piper` |
| Engine | [faster-whisper](https://github.com/SYSTRAN/faster-whisper) (Whisper on CTranslate2, int8) with [Silero VAD](https://github.com/snakers4/silero-vad) | [Piper](https://github.com/OHF-Voice/piper1-gpl) voices (ONNX) |
| Compared with | `stt_whisper`: the same models, faster (it also runs well on a CPU), and a neural voice detector that copes better with noise | `tts_mms`: more natural voices, a CPU is enough (the GPU stays free), the voice loads in about a second |

```bash
ros2 launch simple_hri simple_hri.launch.py stt:=faster_whisper
ros2 launch simple_hri simple_hri.launch.py backend:=ollama stt:=faster_whisper
ros2 launch simple_hri simple_hri.launch.py piper_voice:=es_ES-davefx-medium     # another voice
ros2 launch simple_hri simple_hri.launch.py tts:=mms                             # MMS-TTS
```

**Install** their Python packages (in `requirements.txt`): `pip install faster-whisper piper-tts`
(PyTorch, already required, runs Silero VAD).

**Models.** Nothing to download by hand: the first launch downloads them.
- The faster-whisper model (`whisper_model`) goes to `models/`.
- Silero VAD goes to `~/.cache/torch/hub` (needs Internet the first time).
- The Piper voice (`<voice>.onnx` and `<voice>.onnx.json`) goes to `models/`, or to
  `SIMPLE_HRI_MODELS_DIR` if it is set. A voice can also be copied there by hand, e.g. to use
  one outside the catalog or on a robot without Internet.

On a computer without Internet, download the voices beforehand:

```bash
python3 -c "from simple_hri.tts_piper import download_voice, voice_path; \
  [download_voice(voice_path(v, 'models')) for v in ('es_ES-sharvard-medium', 'en_US-lessac-medium')]"
```

**Speed.** On a laptop with an RTX A500 (4 GB), for a 6-7 s sentence: Piper synthesizes it in
0.15 s on the CPU (MMS-TTS: 0.17-0.19 s on the GPU, 1.4 s on the CPU); faster-whisper `small`
transcribes it in 0.36-0.41 s on the GPU and 1.1-1.3 s on the CPU (Whisper `small`:
0.50-0.62 s on the GPU, 2.0-3.6 s on the CPU), with the same text.

**Parameters** of the nodes (besides those of [Nodes](#nodes)):

| Node | Parameter | Default | Description |
| --- | --- | --- | --- |
| `stt_faster_whisper` | `device` | `cpu` | `cpu`, `cuda` or `auto` (the GPU if there is one). The launch file passes `stt_device` |
| | `compute_type` | `auto` | `int8_float32` on the GPU, `int8` on the CPU |
| | `vad_threshold` | `0.5` | Silero speech probability that counts as voice |
| | `grace_period` | `0.8` | Seconds of silence that end the phrase |
| | `max_phrase_secs` | `15.0` | Longest phrase recorded (s) |
| | `pre_record` | `0.5` | Seconds kept before the voice, so that the first syllable is not cut |
| | `input_device` | `''` | Microphone (`sounddevice`); empty = the default one |
| | `audio_file` | `''` | Tests only: a WAV file read instead of the microphone, at real-time pace |
| `tts_piper` | `voice` | `''` | As `piper_voice` above |
| | `models_dir` | `''` | Where the voices are; empty = `SIMPLE_HRI_MODELS_DIR` or `./models` |
| | `download` | `true` | Download a missing voice of the catalog |
| | `length_scale` | `1.0` | Below 1 speaks faster, above 1 slower |
| | `speaker` | `-1` | As `piper_speaker` above |
| | `noise_scale` | `-1.0` | Variation of the voice (more expressive above the voice's value, flatter below). Negative = the voice's value |
| | `noise_w_scale` | `-1.0` | Variation of the phoneme durations (rhythm). Negative = the voice's value |

### Mixing engines

`stt`, `tts`, `extract` and `yesno` choose the engine of one capability, instead of the one of
`backend`:

```bash
# OpenAI STT (better transcriptions), everything else local
ros2 launch simple_hri simple_hri.launch.py stt:=openai
# Cloud, but with the local TTS (free)
ros2 launch simple_hri simple_hri.launch.py backend:=cloud tts:=mms
# Local, but Extract and Yes/No in Ollama
ros2 launch simple_hri simple_hri.launch.py extract:=ollama yesno:=ollama ollama_host:=server
```

Each capability needs only the keys of its own engine (`stt:=openai` needs `OPENAI_API_KEY`,
but not the Google credentials). The nodes can also be run one by one with `ros2 run` (see
[Nodes](#nodes)).

## Testing

With `simple_hri` running, in another terminal (with the workspace sourced and the same
`ROS_DOMAIN_ID`):

**Extract and Yes/No.** `test_llm` sends 20 sentences to each service, compares the answers
with the expected ones and prints how many are correct and how long each request takes:

```bash
ros2 run simple_hri test_llm            # both services
ros2 run simple_hri test_llm extract    # only Extract
ros2 run simple_hri test_llm yesno      # only Yes/No
ros2 run simple_hri test_llm en         # English sentences (launch with language:=en)
```

Use it to compare backends and models: launch `simple_hri` with one of them, run `test_llm`,
and repeat with the next one.

Extract with the current prompts (November 2026): the 20 sentences of `test_llm`, and 11
requests on orders with a drink, a food and a dessert in the same sentence ("quiero una coca
cola, unos macarrones y de postre un flan", asking for `bebida`, `comida` or `postre`), which
need the model to separate the categories:

| `backend` | Model | Extract es / en | Orders es / en | Time per request |
| --- | --- | --- | --- | --- |
| `ollama` | `qwen2.5:7b` | 20 / 20 | 11 / 11 | 1.3-2.4 s (laptop CPU) |
| `ollama` | `qwen2.5:3b` | 14 / - | 6 / - | 1.0 s (laptop CPU) |
| `local` | `Qwen2.5-1.5B-Instruct` | 14 / 20 | 10 / 5 | 0.3 s (4 GB laptop GPU) |

With the previous prompts the small models scored better on the 20 sentences (`qwen2.5:3b`
19, `Qwen2.5-1.5B` 16), but none of them separated the categories of an order (2-4 of 11,
also `qwen2.5:7b`).

Yes/No with the current prompts (November 2026), in Spanish: the 20 sentences of `test_llm`,
24 colloquial answers to "¿Estás bien?" ("estoy de putísima madre", "no me puedo quejar",
"regular, tirando a mal") and 12 more that are not in the prompt, all written as the STT
writes them (capital letter and final period):

| `backend` | Model | `test_llm` | Colloquial | Not in the prompt |
| --- | --- | --- | --- | --- |
| `ollama` | `qwen2.5:7b` | 19 / 20 | 22 / 24 | 12 / 12 |
| `ollama` | `qwen2.5:3b` | 19 / 20 | 23 / 24 | 12 / 12 |
| `local` | `Qwen2.5-1.5B-Instruct` | 19 / 20 | 22 / 24 | 11 / 12 |

With the previous prompt the colloquial answers failed (7-17 of 24): it only knew yes/no as
accepting or refusing, and `qwen2.5:7b` answered `UNKNOWN` to "estoy bien". Previous results (October 2026, previous prompts), with the Spanish
(`test_llm`) and English (`test_llm en`) sentences:

| `backend` | Model | Extract es / en | Yes/No es / en | Time per request |
| --- | --- | --- | --- | --- |
| `cloud` | `gpt-4.1-mini` | 20 / 20 | 20 / 20 | 0.8 s |
| `ollama` | `qwen2.5:3b` | 19 / 19 | 20 / 18 | 0.03 s (server GPU) |
| `ollama` | `gemma3:4b` | 18 / - | 20 / - | - |
| `ollama` | `qwen2.5vl:32b` | 18 / - | 20 / - | - |
| `ollama` | `llama3.2:3b` | 18 / - | 18 / - | - |
| `local` | `Qwen2.5-1.5B-Instruct` | 16 / 20 | 19 / 19 | 0.3 s (4 GB laptop GPU), 10-30 s (CPU) |
| `local` | `Qwen2.5-0.5B-Instruct` | 8 / - | 17 / - | ~8 s (CPU) |
| `yesno:=keywords` | Word lists | - | 20 / 20 | 0 s |

The word lists were completed with these same sentences, so their 20/20 is optimistic: with
other answers they fail more than an LLM. `hf` and the cells marked `-` were not measured.

**Single requests:**

```bash
ros2 service call /extract_service simple_hri_interfaces/srv/Extract \
  "{interest: 'bebida', text: 'me apetece una coca cola bien fría'}"
ros2 service call /yesno_service simple_hri_interfaces/srv/YesNo "{text: 'sí, claro'}"
```

**STT and TTS** (speak after launching the STT ones):

```bash
ros2 action send_goal --feedback /tts_action simple_hri_interfaces/action/Say "{text: 'Hola'}"
ros2 action send_goal --feedback /stt_action simple_hri_interfaces/action/Listen "{max_wait: 8.0}"
ros2 run simple_hri test_voice          # listens and repeats what you said
ros2 run simple_hri test_extract        # sends a sample sentence to Extract
```

While testing, the terminal of the launch shows what each node receives and answers.

## Ollama: LLM on your GPU or on a server

[Ollama](https://ollama.com) serves LLMs on a GPU (NVIDIA, AMD or Apple) and falls back to
the CPU when there is none. The nodes `extract_ollama` and `yesno_ollama`
send their requests to an Ollama server, which can run on your computer or on another
machine. STT and TTS do not use Ollama: they always run on the computer where you launch
`simple_hri`.

```text
 your computer                                   Ollama server (the same computer or another one)
 ┌──────────────────────────────────┐            ┌───────────────────────────────┐
 │ stt_whisper   (Whisper)          │            │ ollama serve  (port 11434)    │
 │ tts_piper     (Piper)            │   HTTP     │   └─ model on the GPU         │
 │ extract_ollama  ─────────────────┼───────────►│      (qwen2.5:7b, gemma3...)  │
 │ yesno_ollama    ─────────────────┼───────────►│                               │
 └──────────────────────────────────┘            └───────────────────────────────┘
```

The steps are the same for both cases: install Ollama and download a model where it will
run (step 1 and 2), and then point the nodes to it (step 3a for your computer, 3b for a
server).

### 1. Install Ollama

Do this on the machine with the GPU: your computer or the server.

**GPU drivers.** For an NVIDIA GPU you only need the NVIDIA driver: `nvidia-smi` must show
the GPU. For an AMD GPU, install ROCm and the ROCm package of Ollama (see the
[Linux instructions](https://docs.ollama.com/linux)). Without a GPU Ollama works on the
CPU, but only small models (3-4B) answer in a reasonable time.

**Linux** (installs the `ollama` command and a systemd service, `ollama`, that starts at
boot):

```bash
curl -fsSL https://ollama.com/install.sh | sh
```

Check that it is running and that it found the GPU:

```bash
systemctl status ollama
curl http://localhost:11434/api/version
journalctl -u ollama --no-pager | grep -i -E "gpu|cuda|rocm"   # lists the GPU detected
```

Run the same script again to update Ollama. On **macOS and Windows**, download the
application from [ollama.com/download](https://ollama.com/download).

**Docker** (alternative, for example on a server where you prefer not to install anything).
The GPU needs the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html):

```bash
# NVIDIA GPU. 127.0.0.1: the port is reachable only from this machine (see step 3b)
docker run -d --gpus=all -v ollama:/root/.ollama -p 127.0.0.1:11434:11434 \
  --name ollama --restart unless-stopped ollama/ollama
# AMD GPU: --device /dev/kfd --device /dev/dri instead of --gpus=all, and image ollama/ollama:rocm
```

With Docker, run the `ollama` commands of the next steps inside the container:
`docker exec -it ollama ollama pull qwen2.5:7b`.

### 2. Download a model

```bash
ollama pull qwen2.5:7b
ollama list                       # downloaded models
ollama run qwen2.5:7b "Hola"      # quick test (Ctrl+D to exit)
```

The models are stored in `/usr/share/ollama/.ollama/models` (Linux service) and take several
GB each. Set `OLLAMA_MODELS` in the service (as `OLLAMA_HOST` in step 3b) to store them
elsewhere.

**Choosing a model.** The services need short, exact answers (in Spanish or English), so an
instruction-tuned model works better than a bigger generic one. Approximate GPU memory
with the default 4-bit quantization:

| Model | GPU memory | Notes |
| --- | --- | --- |
| `qwen2.5:3b` | ~2.5 GB | Fits in 4 GB GPUs and answers several times faster, but it misses: it mixes up the categories when a sentence has several (drink, food and dessert in one order), and even simple cases fail ("quiero un café con leche y una tostada", `bebida` → `NONE`). Use it only if speed matters more than the answers |
| `llama3.2:3b` | ~2.5 GB | Alternative of the same size |
| `gemma3:4b` | ~4 GB | Better Spanish, tight in a 4 GB GPU |
| `qwen2.5:7b` (default) | ~5 GB | The smallest that separates several categories in one sentence. 8 GB GPUs; on a smaller one it runs on the CPU, at about 2 s per request |
| `llama3.1:8b` | ~6 GB | 8 GB GPUs |
| `gemma3:12b`, `qwen2.5:14b` | ~9-10 GB | Servers, 12-16 GB GPUs |
| `qwen2.5:32b`, `gemma3:27b` | ~20 GB | Servers, 24 GB GPUs or more |

If a model does not fit in the GPU memory, Ollama splits it between GPU and CPU, which is
much slower. Prefer non-reasoning models: reasoning models (`qwen3`, `deepseek-r1`...)
think before answering, which is slower and may use up the token limit of the answer.

### 3a. Use Ollama on your computer

With Ollama running and the model downloaded, nothing else is needed: the nodes connect to
`localhost:11434` by default.

```bash
ros2 launch simple_hri simple_hri.launch.py backend:=ollama                        # qwen2.5:7b
ros2 launch simple_hri simple_hri.launch.py backend:=ollama ollama_model:=gemma3:4b
```

Check where the model runs with `ollama ps` (`PROCESSOR` column): `100% GPU` is the
fastest; `100% CPU` also works with `qwen2.5:7b` (about 2 s per request); `CPU/GPU` means
that it does not fit in the GPU and is split, which can be slower than the CPU alone. On laptops with hybrid graphics, Ollama uses
the NVIDIA GPU directly through CUDA, with no PRIME offload variables.

### 3b. Use Ollama on a server

Install Ollama and download the models **on the server** (steps 1 and 2). By default Ollama
listens only on `127.0.0.1`, so it cannot be reached from other machines. There are two ways
to connect from your computer:

**Option A: SSH tunnel (recommended).** Nothing changes on the server, the traffic is
encrypted and only users with an SSH account can use it. On your computer:

```bash
ssh -N -L 11434:127.0.0.1:11434 user@server     # leave it open (or add -f to run it in background)
ros2 launch simple_hri simple_hri.launch.py backend:=ollama ollama_model:=qwen2.5:14b
```

`localhost:11434` is now the server, so the nodes need no `ollama_host`. If Ollama is also
running on your computer, port 11434 is in use: choose another local port.

```bash
ssh -N -L 11435:127.0.0.1:11434 user@server
ros2 launch simple_hri simple_hri.launch.py backend:=ollama ollama_host:=localhost:11435 ollama_model:=qwen2.5:14b
```

**Option B: open the port on the server.** Ollama has **no authentication**: anyone who
reaches the port can use the GPU. Do it only in a trusted network, and restrict the
port with the firewall. On the server:

```bash
sudo systemctl edit ollama
```

Add these lines in the editor that opens, save and restart the service:

```ini
[Service]
Environment="OLLAMA_HOST=0.0.0.0:11434"
```

```bash
sudo systemctl restart ollama
sudo ufw allow from 192.168.1.0/24 to any port 11434 proto tcp   # only your network (if ufw is used)
```

With Docker, publish the port with `-p 11434:11434` instead of `-p 127.0.0.1:11434:11434`.
Then, on your computer:

```bash
curl http://server:11434/api/version            # the server is reachable
ros2 launch simple_hri simple_hri.launch.py backend:=ollama ollama_host:=server ollama_model:=qwen2.5:14b
```

Instead of `ollama_host`, you can `export OLLAMA_HOST=server:11434`: the nodes use it when
`ollama_host` is `auto` (the default), and so does the `ollama` command (`ollama list` shows then the
models of the server).

**Server settings.** Other variables of the service, added in the same `[Service]` section:

| Variable | Default | Use |
| --- | --- | --- |
| `OLLAMA_KEEP_ALIVE` | `5m` | How long a model stays in memory after the last request |
| `OLLAMA_NUM_PARALLEL` | `1` | Requests a model answers at the same time (several robots or students) |
| `OLLAMA_MAX_LOADED_MODELS` | depends on the memory | Models loaded at the same time, if they fit |
| `OLLAMA_MODELS` | `/usr/share/ollama/.ollama/models` | Directory of the models |

`journalctl -u ollama -f` shows the requests that arrive.

### Keeping the model loaded

Ollama unloads a model 5 minutes after its last request, and loading it again takes from a
few seconds (small model) to a minute (big model on a server). The robot would then take
long to answer after a pause. The nodes load the model at startup (`preload`), and
`ollama_keep_alive` sets how long it stays loaded after each request:

```bash
ros2 launch simple_hri simple_hri.launch.py backend:=ollama ollama_keep_alive:=1h
ros2 launch simple_hri simple_hri.launch.py backend:=ollama ollama_keep_alive:=-1    # until Ollama stops
```

On a shared server, use a limited time rather than `-1`: a loaded model keeps its GPU
memory for itself.

### Node parameters

Both Ollama nodes accept:

| Parameter | Default | Description |
| --- | --- | --- |
| `host` | `''` | Ollama server (`server`, `server:11434`, `http://server:11434`). Empty = `OLLAMA_HOST` or `http://localhost:11434` |
| `model` | `qwen2.5:7b` | Ollama model |
| `keep_alive` | `''` | How long the model stays loaded after each request (`10m`, `1h`, `-1`). Empty = server default |
| `prompt_file` | `prompts_es.yaml` | Prompts file: a name in `params/` or an absolute path |
| `timeout` | `60.0` | Seconds to wait for each answer |
| `preload` | `true` | Load the model when the node starts, so that the first request is fast |

At startup the nodes check that the server is reachable and that it has the model, and log
how to fix it otherwise (`ollama pull <model>`). They start anyway, and the requests return
`ERROR: ...` until the server is available.

To combine Ollama with other STT and TTS engines, use the arguments of each capability (see
[Mixing engines](#mixing-engines)), e.g. `extract:=ollama yesno:=ollama stt:=openai`.

## Actions

Listening and speaking are **actions**: they last seconds, give feedback, can be canceled and,
for speaking, finish when the audio really ends. Each node serves one goal at a time (two goals
never use the microphone or the speakers at once).

### Listen

**Action:** `/stt_action` (`simple_hri_interfaces/action/Listen`)

- **Goal:** `max_wait`: seconds to wait for the person to start speaking (`0` = default, 10 s).
- **Feedback:** `status`: `listening`, `speech_detected`, `transcribing`.
- **Result:** `success`, `timed_out` (nobody spoke), `text`, `message`. The text is also
  published on `/listened_text` (`std_msgs/msg/String`). It records until 1 s of silence
  (1.5 s with the cloud backend, 0.8 s with faster-whisper) or 15 s, in the language of the
  launch (`language`).
- **Noisy rooms:** webrtcvad (the voice detector of `stt_whisper` and `stt_openai`) takes
  room noise for voice: in a classroom with people talking in groups it marks every frame as
  voice, and the phrase never ended. Now (`simple_hri/vad_phrase.py`) a frame is voice only
  if it is also `noise_margin` dB louder than the background, measured while waiting for the
  person (launch argument `background_noise`: `low` 4, `medium` 6, `high` 8); the phrase starts when most of 0.3 s is voice and ends when little
  of the last second is (a click or somebody else's word does not keep it open), and it
  never lasts more than 15 s. The person must speak closer to the microphone than the noise.
- **Noise is not speech:** with noise or silence, Whisper often invents a sentence
  ("¡Suscríbete!", "Subtítulos por la comunidad de Amara.org"). Those transcriptions are
  discarded (`simple_hri/stt_filter.py`): the segments that Whisper itself rates as probably
  not speech, an empty or punctuation-only text, and the known hallucinations. The node then
  keeps listening for the rest of `max_wait`, so `timed_out` means that nobody really spoke.
  The parameter `hallucination_filter` of the STT nodes (default `true`) turns it off.
- **Cancel:** stops recording immediately.

### Say

**Action:** `/tts_action` (`simple_hri_interfaces/action/Say`)

- **Goal:** `text`.
- **Feedback:** `remaining`: seconds of playback left.
- **Result:** `success`, `message`. It finishes when playback really ends (exact
  duration, read from the generated WAV).
- **Cancel:** stops the audio (not possible with `play_sound:=false`, where the audio is
  sent to another device).

The nodes spin with a multi-threaded executor, so a cancel request is attended while the
action is running.

## Services

Extracting information and yes/no detection are **services**: queries on a text (normally the
one `/stt_action` returned), answered at once.

### Extract

**Service:** `/extract_service` (`simple_hri_interfaces/srv/Extract`)

It does not record audio: get the text first with STT.

- **Request:** `interest` (what to look for, e.g. `bebida`, `nombre`) and `text`.
- **Response:** `result`: the relevant words (several ones separated by `;`), `NONE` if
  there is nothing relevant (or the text is empty), or a string starting with `ERROR` on
  failure. The result is also published on `/extracted_text`.

```python
from simple_hri_interfaces.srv import Extract

extract_client = self.create_client(Extract, '/extract_service')
request = Extract.Request()
request.interest = 'bebida'
request.text = 'me apetece una coca cola'
future = extract_client.call_async(request)
# When done: future.result().result == 'coca cola'
```

### Yes/No

**Service:** `/yesno_service` (`simple_hri_interfaces/srv/YesNo`)

It does not record audio either.

- **Request:** `text`.
- **Response:** `result`: `YES`, `NO` or `UNKNOWN` (doubt, or an answer that is neither), or
  a string starting with `ERROR` on failure. The result is also published on `/yesno_result`.

```python
from simple_hri_interfaces.srv import YesNo

yesno_client = self.create_client(YesNo, '/yesno_service')
request = YesNo.Request()
request.text = 'sí, claro'
future = yesno_client.call_async(request)
# When done: future.result().result == 'YES'
```

### Audio forwarding

The models can run on one computer and the robot speak through the speakers of another one
(for example, the robot itself), in the same ROS 2 network (`ROS_DOMAIN_ID`):

```bash
# Computer with the models
ros2 launch simple_hri simple_hri.launch.py tts_speaks:=false
# Computer with the speakers
ros2 launch simple_hri simple_hri.launch.py run_interaction_services:=false
```

```text
 computer with the models                              computer with the speakers
 tts_* ──WAV file──► audio_service ──/audio_file_data──► audio_file_player ──► speakers
        /trigger_audio_send          (UInt8MultiArray)
```

With `tts_speaks:=false`, TTS does not play the WAV file: it calls `/trigger_audio_send`
(`audio_send_interfaces/srv/SendAudio`, with the file path). `audio_service`, launched on the
same computer because it reads the file, publishes its bytes on `/audio_file_data`
(`std_msgs/msg/UInt8MultiArray`). On the other computer, `run_interaction_services:=false`
launches only `audio_file_player`, which plays them. The Say action cannot stop the audio
in this mode.

## Nodes

One node per capability and engine, named `<capability>_<engine>`. The launch file chooses
and configures them; to run one by hand, see the end of the table.

| Node | Capability | Engine | Needs | Parameters |
| --- | --- | --- | --- | --- |
| `stt_whisper` | STT | Whisper, local | - | `model` (`small`), `language` (`es`), `device` (`cpu`, `cuda` or `auto`), `hallucination_filter` (`true`), `noise_margin` (`6.0`), `grace_period` (`1.0`), `max_phrase_secs` (`15.0`), `vad_mode` (`2`) |
| `stt_faster_whisper` | STT | faster-whisper with Silero VAD, local | - | `model` (`small`), `language` (`es`), `device` (`cpu`, `cuda` or `auto`), `compute_type` (see [faster-whisper and Piper](#faster-whisper-and-piper)), `hallucination_filter` (`true`) |
| `stt_openai` | STT | OpenAI `whisper-1` | `OPENAI_API_KEY` + credits | `language` (`es`), `hallucination_filter` (`true`; only the known phrases, the API gives no no-speech probability), `noise_margin` (`6.0`), `grace_period` (`1.5`), `max_phrase_secs` (`15.0`), `vad_mode` (`2`) |
| `tts_mms` | TTS | `facebook/mms-tts-<lang_code>`, local | - | `lang_code`, `volume`, `use_gpu`, `play_sound`, `audio_player` |
| `tts_piper` | TTS | Piper voice, local (CPU) | - | `lang_code` (`es`/`en`), `voice`, `speaker`, `models_dir`, `volume`, `length_scale`, `noise_scale`, `noise_w_scale`, `play_sound`, `audio_player` |
| `tts_google` | TTS | Google Cloud TTS | `GOOGLE_APPLICATION_CREDENTIALS` + billing | `language` (`en`), `voice` (from `language`), `play_sound`, `audio_player` |
| `llm_local` | Extract and Yes/No | LLM run with transformers (`Qwen/Qwen2.5-1.5B-Instruct`), loaded once for both services. `extract_local` and `yesno_local` run it with only one service | - | `model_id`, `use_gpu`, `prompt_file`, `extract`, `yesno` |
| `extract_ollama` | Extract | Ollama | Ollama server | `host`, `model`, `keep_alive`, `prompt_file`, `timeout`, `preload` |
| `extract_hf` | Extract | Hugging Face Inference Providers (`meta-llama/Meta-Llama-3-8B-Instruct`) | `HF_TOKEN` + credits | `model_id`, `prompt_file` |
| `extract_openai` | Extract | OpenAI (`gpt-4.1-mini`) | `OPENAI_API_KEY` + credits | `model`, `prompt_file` |
| `yesno_keywords` | Yes/No | Word lists of `language` (optionally a Hugging Face LLM, with `use_llm:=true`) | `HF_TOKEN` + credits only with `use_llm` | `language` (`es`), `use_llm`, `model_id`, `prompt_file` |
| `yesno_ollama` | Yes/No | Ollama | Ollama server | `host`, `model`, `keep_alive`, `prompt_file`, `timeout`, `preload` |
| `yesno_hf` | Yes/No | Hugging Face Inference Providers (`meta-llama/Meta-Llama-3-8B-Instruct`) | `HF_TOKEN` + credits | `model_id`, `prompt_file` |
| `yesno_openai` | Yes/No | OpenAI (`gpt-4.1-mini`) | `OPENAI_API_KEY` + credits | `model`, `prompt_file` |
| `audio_service` | `/trigger_audio_send` | Publishes a WAV file on `/audio_file_data` | - | - |
| `audio_file_player` | - | Plays the WAV files received on `/audio_file_data` | - | `audio_player` |

Run any of them with `ros2 run simple_hri <node> --ros-args -p <param>:=<value>`, for example
`ros2 run simple_hri extract_ollama --ros-args -p host:=server -p model:=gemma3:4b`.

### Prompts

Every LLM backend (local, Ollama, Hugging Face, OpenAI) uses the same prompts, one file per
language in `params/`: `prompts_es.yaml` and `prompts_en.yaml`. Each one has, for each service:

- `system`: the instructions for the model, with rules and examples. Extract answers the
  values separated by `;`, or `NONE`; Yes/No answers `YES`, `NO` or `UNKNOWN`.
- `user`: the message sent with each request, where `{interest}` and `{text}` are replaced
  by the fields of the request. It has the same format as the examples of `system`, because
  small models copy that format to answer: change both together.

```yaml
extract:
  system: |
    Eres un extractor de información...
    Frase: "hola, soy Luis Miguel" | Categoría: nombre -> Luis Miguel
  user: 'Frase: "{text}" | Categoría: {interest} ->'
```

The examples matter for small models (0.5-4B parameters): without them, `qwen2.5:3b` missed
names such as "soy María José" and answered `NO` to "vale". The text of a Yes/No request is
sent in lowercase and without a final `.` or `!` (`clean_yesno_text`): `Qwen2.5-1.5B`
answered `NO` to "Estoy de puta madre." and `YES` to "estoy de puta madre". Extract sends
the text as it is, because it copies the values from it. `simple_hri/prompts.py` loads the
file and cleans the answers of every backend in the same way.

To change the prompts, copy the file, edit it and pass it with `prompt_file` (a name in
`params/` or an absolute path); then check the result with `test_llm` (see
[Testing](#testing)):

```bash
ros2 launch simple_hri simple_hri.launch.py backend:=ollama prompt_file:=/home/me/my_prompts.yaml
```

A new file in `params/` (or an edited one, if you build without `--symlink-install`) needs a
rebuild of the workspace to be installed.

## Troubleshooting

### "API key not found" / missing credentials

Export the variables in the terminal where you launch the nodes:

```bash
echo $OPENAI_API_KEY
echo $GOOGLE_APPLICATION_CREDENTIALS
echo $HF_TOKEN
```

### `429 insufficient_quota` (OpenAI) / `402 Payment Required` (Hugging Face)

The key or token is valid, but the account has no credits: add them in the
[OpenAI billing page](https://platform.openai.com/settings/organization/billing) or the
[Hugging Face billing page](https://huggingface.co/settings/billing), or switch to a free
backend (`backend:=local` or `backend:=ollama`).

### "Ollama server not reachable" / "Model not found"

```bash
curl http://localhost:11434/api/version   # is the server (or the tunnel) up?
ollama list                                # models downloaded (run it on the server)
ollama pull qwen2.5:7b
```

If the SSH tunnel says `bind: Address already in use`, Ollama is also running on your
computer: use another local port (`-L 11435:127.0.0.1:11434` and `ollama_host:=localhost:11435`).

### Slow answers from Ollama

`ollama ps` on the Ollama machine: `CPU/GPU` or `100% CPU` in `PROCESSOR` means the model
does not fit in the GPU, so choose a smaller one. If only the first answer after a pause is
slow, the model was unloaded: set `ollama_keep_alive`.

### "Service not available" / "Action server not available"

The launch is not running, it is still loading the models (wait for the `ready` messages), or
the terminals have different `ROS_DOMAIN_ID`:

```bash
ros2 action list                                 # /stt_action, /tts_action
ros2 service list | grep -E "(extract|yesno)"
```

### `CUDA out of memory`

The local models do not fit in the GPU. With the defaults, only the local LLM uses the GPU
(about 3.1 GB). `stt_device:=gpu` adds Whisper (about 1.5 GB with `small`) and `tts:=mms`
adds MMS-TTS (about 0.3 GB), which do not fit in a 4 GB GPU with the LLM; another program
using the GPU can also make it fail. Use `llm_device:=cpu` (slow), a
smaller `local_model`, or move the LLM to Ollama (`backend:=ollama`).

### STT does not hear anything ("No se detectó audio o timeout")

Look at the log of the launch:

- `Timeout: nobody spoke (background -26 dBFS)`: nobody spoke within 10 s, or the microphone
  is muted or not the selected one (desktop sound settings), or the voice was not
  louder enough than the background. With background noise (a class working in groups is
  about -26 dBFS with a laptop microphone), speak closer to the microphone, or launch with
  `background_noise:=low` (more noise is then taken for voice). If noise is taken for voice
  (`Voice detected` with nobody speaking, phrases of 15 s), use `background_noise:=high`.
- `Microphone error ... Invalid sample rate`: the microphone cannot be opened at 16 kHz. It
  happens in conda/pixi environments, whose own ALSA library does not see PipeWire or
  PulseAudio and opens the raw sound card. Point it to the system configuration and plugins:

  ```bash
  export ALSA_CONFIG_DIR=/usr/share/alsa ALSA_CONFIG_PATH=/usr/share/alsa/alsa.conf
  export ALSA_PLUGIN_DIR=/usr/lib/x86_64-linux-gnu/alsa-lib
  ```

Check which devices `sounddevice` sees (with PipeWire there must be a `default` or `pipewire`
one):

```bash
python3 -c "import sounddevice as sd; print(sd.query_devices())"
```

### STT "hears" sentences nobody said

With room noise and nobody speaking, Whisper can return "Subtítulos por la comunidad de
Amara.org", "¡Suscríbete!" or "...". The STT nodes discard them (the log shows
`Discarded, not speech` or `Descartado, no es voz`) and keep listening. If one still gets
through, add it to `HALLUCINATIONS` in `simple_hri/stt_filter.py`. If a real, short answer is
discarded, check the log and, to compare, launch with the filter off:
`ros2 run simple_hri stt_faster_whisper --ros-args -p hallucination_filter:=false`.

### No sound from TTS

Install `aplay` (`sudo apt install alsa-utils`) or choose another player with
`audio_player:=pw-play`.

## Repository Structure

```text
simple_hri/
├── audio_send_interfaces/        # SendAudio service
├── simple_hri/                   # Main Python package
│   ├── simple_hri/               # Nodes: <service>_<backend>.py
│   │   ├── llm_local.py          # Local LLM: Extract and Yes/No with one model
│   │   ├── stt_faster_whisper.py # STT with faster-whisper and Silero VAD
│   │   ├── vad_phrase.py         # Phrase detection with webrtcvad (stt_whisper, stt_openai)
│   │   ├── tts_piper.py          # TTS with Piper voices (and download_voice)
│   │   ├── prompts.py            # Prompt loading and answer cleaning shared by the LLM backends
│   │   ├── ollama_llm.py         # Ollama client shared by the *_ollama nodes
│   │   └── test_llm.py           # Test of the Extract and Yes/No services
│   ├── launch/                   # simple_hri.launch.py (backend:=...)
│   ├── params/                   # LLM prompts: prompts_es.yaml, prompts_en.yaml
│   ├── requirements.txt          # Python dependencies
│   ├── package.xml
│   └── setup.py
├── simple_hri_interfaces/        # Listen and Say actions, Extract and YesNo services
└── README.md                     # This file
```

## Contributing

Contributions are welcome. Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## Authors

- **Rodrigo Pérez** - [rodperex](https://github.com/rodperex)
- **Antonio Bono**

## License

This project is licensed under the Apache-2.0 License - see the [LICENSE](LICENSE) file for details.

## References

- [OpenAI Whisper](https://openai.com/research/whisper)
- [Google Cloud Text-to-Speech](https://cloud.google.com/text-to-speech)
- [Ollama](https://ollama.com) and its [OpenAI compatibility](https://docs.ollama.com/api/openai-compatibility)
- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/index.html)
