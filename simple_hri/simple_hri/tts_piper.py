#!/usr/bin/env python3

# Copyright 2026 Rodrigo Pérez-Rodríguez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
TTS with Piper voices: the same interface as tts_mms.

  /tts_service  (simple_hri_interfaces/Speech)  synthesize a text and play it
  /tts_action   (simple_hri_interfaces/Say)     the same, finishing when the playback ends

Piper voices are small neural models (ONNX) with natural speech that synthesize much faster
than real time on a CPU. The voice is loaded once. A voice is given by its file name and
looked up in models_dir; a missing voice of the Piper catalog is downloaded there the first
time (from huggingface.co/rhasspy/piper-voices). With play_sound false the audio is sent to
/trigger_audio_send instead of being played, as tts_mms does.

The engine comes from the speech_io package of DialoStack (text_to_speech_node).
"""

import os
import threading
import urllib.request
import uuid

import numpy as np
import rclpy
import scipy.io.wavfile
from piper import PiperVoice, SynthesisConfig
from rclpy.node import Node

from audio_send_interfaces.srv import SendAudio
from simple_hri.audio_player import AudioPlayer
from simple_hri.voice_actions import SayActionServer, spin_multithreaded
from simple_hri_interfaces.srv import Speech

# The default voice of each language (ISO 639-1 or the MMS code), from the Piper catalog
DEFAULT_VOICES = {
    'es': 'es_ES-sharvard-medium',
    'spa': 'es_ES-sharvard-medium',
    'en': 'en_US-lessac-medium',
    'eng': 'en_US-lessac-medium',
}
VOICES_URL = 'https://huggingface.co/rhasspy/piper-voices/resolve/main'


def default_models_dir():
    """Where the voices are kept: SIMPLE_HRI_MODELS_DIR, or ./models as the other backends."""
    return os.environ.get('SIMPLE_HRI_MODELS_DIR') or os.path.abspath(
        os.path.join(os.getcwd(), 'models'))


def voice_path(voice, models_dir):
    """The .onnx file of a voice: an absolute path, or a name/file name in models_dir."""
    if os.path.isabs(voice):
        return voice
    name = voice if voice.endswith('.onnx') else voice + '.onnx'
    return os.path.join(models_dir, name)


def download_voice(path, logger=None):
    """Download a voice of the Piper catalog (its .onnx and .onnx.json) to path."""
    name = os.path.basename(path)[:-len('.onnx')]  # e.g. es_ES-sharvard-medium
    lang_region, speaker, quality = name.split('-', 2)
    lang = lang_region.split('_')[0]
    base = f'{VOICES_URL}/{lang}/{lang_region}/{speaker}/{quality}/{name}'
    os.makedirs(os.path.dirname(path), exist_ok=True)
    for suffix in ('.onnx', '.onnx.json'):
        target = path[:-len('.onnx')] + suffix
        if os.path.isfile(target):
            continue
        if logger:
            logger.info(f'Downloading the Piper voice {name}{suffix}...')
        urllib.request.urlretrieve(base + suffix, target + '.part')
        os.replace(target + '.part', target)


class PiperTTSService(Node):

    def __init__(self):
        super().__init__('tts_srv_node')

        self.declare_parameter('lang_code', 'spa')  # language: es / en (or spa / eng)
        self.declare_parameter('voice', '')  # '' = the default voice of lang_code
        self.declare_parameter('models_dir', '')  # '' = SIMPLE_HRI_MODELS_DIR or ./models
        self.declare_parameter('download', True)  # download a missing voice of the catalog
        self.declare_parameter('volume', 1.0)
        self.declare_parameter('length_scale', 1.0)  # < 1 speaks faster, > 1 slower
        # A speaker of a multi-speaker voice (e.g. es_ES-sharvard-medium: 0 or 1); -1 = the voice's
        self.declare_parameter('speaker', -1)
        # Variation of the voice (expressiveness) and of the phoneme durations; < 0 = the voice's
        self.declare_parameter('noise_scale', -1.0)
        self.declare_parameter('noise_w_scale', -1.0)
        self.declare_parameter('use_gpu', False)  # needs onnxruntime-gpu
        self.declare_parameter('play_sound', True)  # False: send the file to /trigger_audio_send
        self.declare_parameter('audio_player', '')  # e.g. 'aplay -q', 'pw-play'. '' = auto

        p = self.get_parameter
        lang_code = p('lang_code').value
        voice = p('voice').value or DEFAULT_VOICES.get(lang_code, DEFAULT_VOICES['en'])
        models_dir = p('models_dir').value or default_models_dir()
        self.volume = p('volume').value
        self.length_scale = p('length_scale').value
        self.speaker = p('speaker').value if p('speaker').value >= 0 else None
        self.noise_scale = p('noise_scale').value if p('noise_scale').value >= 0 else None
        self.noise_w_scale = p('noise_w_scale').value if p('noise_w_scale').value >= 0 else None
        self.play_sound = p('play_sound').value

        path = voice_path(voice, models_dir)
        if not os.path.isfile(path) and p('download').value:
            download_voice(path, self.get_logger())
        self.get_logger().info(f'Loading the Piper voice {path}...')
        self.voice = PiperVoice.load(path, use_cuda=p('use_gpu').value)

        if self.play_sound:
            self.get_logger().info('TTS Service configured to PLAY audio.')
        else:
            self.get_logger().info('TTS Service configured to SEND audio to /trigger_audio_send.')
            self.audio_send_client = self.create_client(SendAudio, '/trigger_audio_send')
        self.player = AudioPlayer(self.get_logger(), p('audio_player').value)

        self.srv = self.create_service(Speech, 'tts_service', self.tts_callback)
        # /tts_action: the same work as the service, but it finishes when the playback ends
        # and can be canceled. The lock serializes the synthesis of the service and the action.
        self.synth_lock = threading.Lock()
        self.say_action = SayActionServer(self, self.speak, self.stop_speaking, self.synth_lock)
        self.get_logger().info('TTSService (Piper) initialized.')

    def tts_callback(self, request, response):
        text = request.text.strip()
        if not text:
            response.success = False
            response.debug = 'Empty text provided'
            self.get_logger().warn(response.debug)
            return response
        self.get_logger().info(f"Processing TTS: '{text[:20]}...'")
        try:
            with self.synth_lock:
                output_path, _ = self.synthesize_and_play(text)
            response.success = True
            response.debug = f'Generated {output_path} via Piper'
        except Exception as e:
            self.get_logger().error(f'TTS Inference/Playback failed: {e}')
            response.success = False
            response.debug = str(e)
        return response

    def synthesize(self, text):
        """The audio of the text: (int16 samples, sample rate)."""
        config = SynthesisConfig(speaker_id=self.speaker, length_scale=self.length_scale,
                                 noise_scale=self.noise_scale, noise_w_scale=self.noise_w_scale,
                                 volume=self.volume)
        chunks = list(self.voice.synthesize(text, syn_config=config))
        if not chunks:
            raise RuntimeError('Piper produced no audio')
        audio = np.concatenate([c.audio_float_array for c in chunks])
        audio = np.clip(audio, -1.0, 1.0)
        return (audio * 32767).astype(np.int16), chunks[0].sample_rate

    def synthesize_and_play(self, text):
        """Synthesize the text, start playing it (non-blocking) and return (path, duration)."""
        audio, rate = self.synthesize(text)
        output_path = os.path.join('/tmp', f'tts_{uuid.uuid4().hex}.wav')
        scipy.io.wavfile.write(output_path, rate=rate, data=audio)
        if self.play_sound:
            self.player.play(output_path)
        elif self.audio_send_client.service_is_ready():
            self.audio_send_client.call_async(SendAudio.Request(file_path=output_path))
        else:
            self.get_logger().warn('Audio send service not available.')
        return output_path, audio.shape[0] / rate

    def speak(self, text):
        """Work of the /tts_action action (SayActionServer takes the lock)."""
        text = text.strip()
        if not text:
            return False, 0.0, 'Empty text provided'
        self.get_logger().info(f"Processing TTS (action): '{text[:20]}...'")
        try:
            output_path, duration = self.synthesize_and_play(text)
            return True, duration, f'Generated {output_path} via Piper'
        except Exception as e:
            self.get_logger().error(f'TTS Inference/Playback failed: {e}')
            return False, 0.0, str(e)

    def stop_speaking(self):
        """Stop the playback (used when the /tts_action goal is canceled)."""
        if self.play_sound:
            self.player.stop()
        else:
            self.get_logger().warn('Audio sent to another device: playback cannot be stopped.')


def main():
    rclpy.init()
    try:
        node = PiperTTSService()
        spin_multithreaded(node)
    except KeyboardInterrupt:  # Ctrl+C while the voice is loading
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
