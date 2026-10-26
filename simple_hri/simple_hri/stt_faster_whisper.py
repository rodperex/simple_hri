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
STT with faster-whisper and Silero VAD: the same interface as stt_whisper.

  /stt_service  (std_srvs/SetBool)               listen once and transcribe
  /stt_action   (simple_hri_interfaces/Listen)   the same, with feedback and cancellation
  /listened_text (std_msgs/String)               every transcription

It listens only while a request is active. Silero VAD (a neural voice detector, more robust
to noise than webrtcvad) finds the speech: the recording starts at the first voice (with
pre_record seconds before it, so that the first syllable is kept) and ends after grace_period
seconds of silence, or after max_phrase_secs. faster-whisper (CTranslate2) transcribes it:
the same Whisper models, several times faster than openai-whisper.

The engines come from the speech_io package of DialoStack (speech_to_text_node).

Parameter audio_file (tests only): a WAV file read instead of the microphone, at real-time
pace, from the start for every request.
"""

import collections
import os
import threading
import time

import ctranslate2
import numpy as np
import rclpy
import sounddevice as sd
import torch
from faster_whisper import WhisperModel
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from simple_hri.voice_actions import ListenActionServer, spin_multithreaded

SAMPLE_RATE = 16000
CHUNK = 512  # samples per Silero VAD window (32 ms at 16 kHz)
MAX_WAIT_SECONDS = 10.0  # longest wait for somebody to start speaking


def resolve_device(device, compute_type, cuda_devices):
    """Turn "auto" device/compute type into concrete faster-whisper values.

    int8_float32 is the CUDA default because it is fast on every NVIDIA GPU; float16 only
    pays off on cards with tensor cores.
    """
    if device == 'auto':
        device = 'cuda' if cuda_devices > 0 else 'cpu'
    if compute_type == 'auto':
        compute_type = 'int8_float32' if device == 'cuda' else 'int8'
    return device, compute_type


class FileStream:
    """Reads a WAV file as if it were the microphone (tests), at real-time pace."""

    def __init__(self, path):
        import scipy.io.wavfile
        import scipy.signal
        rate, data = scipy.io.wavfile.read(path)
        if data.ndim > 1:
            data = data.mean(axis=1)
        if data.dtype == np.int16:
            data = data.astype(np.float32) / 32768.0
        else:
            data = data.astype(np.float32)
        if rate != SAMPLE_RATE:
            data = scipy.signal.resample_poly(data, SAMPLE_RATE, rate).astype(np.float32)
        # Silence after the file, so that the end of the phrase is detected
        self._audio = np.concatenate([data, np.zeros(SAMPLE_RATE * 3, dtype=np.float32)])
        self._pos = 0

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def read(self, frames):
        chunk = self._audio[self._pos:self._pos + frames]
        self._pos += frames
        if len(chunk) < frames:
            chunk = np.concatenate([chunk, np.zeros(frames - len(chunk), dtype=np.float32)])
        time.sleep(frames / SAMPLE_RATE)
        return chunk.reshape(-1, 1), False


class STTFasterWhisper(Node):

    def __init__(self):
        super().__init__('stt_service_node')

        self.declare_parameter('model', 'tiny')  # tiny, base, small, medium, large-v3...
        self.declare_parameter('language', 'es')  # Language spoken: es, en...
        self.declare_parameter('device', 'auto')  # auto, cuda or cpu
        self.declare_parameter('compute_type', 'auto')  # auto, int8, int8_float32, float16...
        self.declare_parameter('cpu_threads', 4)
        self.declare_parameter('input_device', '')  # sounddevice input; '' = default
        self.declare_parameter('vad_threshold', 0.5)  # Silero speech probability
        self.declare_parameter('grace_period', 0.8)  # s of silence that end the phrase
        self.declare_parameter('max_phrase_secs', 15.0)  # s, longest phrase recorded
        self.declare_parameter('pre_record', 0.5)  # s of audio kept before the voice
        self.declare_parameter('audio_file', '')  # tests only: a WAV instead of the microphone

        p = self.get_parameter
        self.language = p('language').value
        self.vad_threshold = p('vad_threshold').value
        self.grace_period = p('grace_period').value
        self.max_phrase_secs = p('max_phrase_secs').value
        self.pre_record = p('pre_record').value
        self.audio_file = p('audio_file').value
        input_device = p('input_device').value
        self.input_device = input_device if input_device else None

        self.get_logger().info('Loading Silero VAD...')
        self.vad_model, _ = torch.hub.load(
            repo_or_dir='snakers4/silero-vad', model='silero_vad',
            force_reload=False, trust_repo=True)
        self.asr = self.load_asr()

        self.srv = self.create_service(SetBool, 'stt_service', self.stt_callback)
        self.pub = self.create_publisher(String, '/listened_text', 10)

        # /stt_action: the same work as the service, with feedback and cancellation.
        # The lock keeps the service and the action from using the microphone at once.
        self.mic_lock = threading.Lock()
        self.last_record_timed_out = False
        self.listen_action = ListenActionServer(self, self.listen, self.mic_lock)

        source = f'file {self.audio_file}' if self.audio_file else 'microphone'
        self.get_logger().info(f'STT (faster-whisper) ready, listening from the {source}.')

    def load_asr(self):
        p = self.get_parameter
        model_name = p('model').value
        auto = p('device').value == 'auto'
        device, compute_type = resolve_device(
            p('device').value, p('compute_type').value, ctranslate2.get_cuda_device_count())
        # The models are kept in ./models, as the other backends do
        cache_dir = os.path.abspath(os.path.join(os.getcwd(), 'models'))
        os.makedirs(cache_dir, exist_ok=True)
        self.get_logger().info(f'Loading faster-whisper ({model_name}, {device}/{compute_type})...')
        try:
            return self.load_whisper(model_name, device, compute_type, cache_dir)
        except (RuntimeError, ValueError) as e:
            if not (auto and device == 'cuda'):
                raise
            self.get_logger().warning(f'GPU unavailable ({e}), using the CPU.')
            return self.load_whisper(
                model_name, *resolve_device('cpu', p('compute_type').value, 0), cache_dir)

    def load_whisper(self, model_name, device, compute_type, cache_dir):
        model = WhisperModel(
            model_name, device=device, compute_type=compute_type,
            cpu_threads=self.get_parameter('cpu_threads').value, download_root=cache_dir)
        if device == 'cuda':
            # Warm up: the first CUDA transcription is much slower
            segments, _ = model.transcribe(np.zeros(SAMPLE_RATE, dtype=np.float32))
            list(segments)
        return model

    def open_stream(self):
        if self.audio_file:
            return FileStream(self.audio_file)
        return sd.InputStream(
            samplerate=SAMPLE_RATE, channels=1, dtype='float32',
            device=self.input_device, blocksize=CHUNK)

    def record_audio_with_vad(self, max_wait=MAX_WAIT_SECONDS, should_stop=None, on_status=None):
        """
        Record one phrase: wait for voice (at most max_wait s), then record until
        grace_period s of silence or max_phrase_secs. Returns float32 audio, empty on timeout
        or cancellation. should_stop() is checked on every chunk and on_status() reports the
        stage ("listening", "speech_detected"); both are optional.
        """
        pre_buffer = collections.deque(maxlen=max(1, int(self.pre_record * SAMPLE_RATE / CHUNK)))
        max_silence = int(self.grace_period * SAMPLE_RATE / CHUNK)
        max_chunks = int(self.max_phrase_secs * SAMPLE_RATE / CHUNK)
        phrase = []
        silence = 0
        start_wait = time.time()
        self.last_record_timed_out = False
        self.vad_model.reset_states()

        self.get_logger().info('Listening... (speak now)')
        if on_status:
            on_status('listening')
        try:
            with self.open_stream() as stream:
                while True:
                    if should_stop and should_stop():
                        self.get_logger().info('Recording canceled.')
                        return np.array([], dtype=np.float32)
                    chunk, overflow = stream.read(CHUNK)
                    if overflow:
                        self.get_logger().warning('Audio overflow')
                    chunk = chunk.flatten().astype(np.float32)
                    speech_prob = self.vad_model(torch.from_numpy(chunk), SAMPLE_RATE).item()
                    is_voice = speech_prob > self.vad_threshold

                    if not phrase:
                        # Waiting for voice
                        pre_buffer.append(chunk)
                        if is_voice:
                            self.get_logger().info('Voice detected, recording...')
                            if on_status:
                                on_status('speech_detected')
                            phrase.extend(pre_buffer)
                        elif time.time() - start_wait > max_wait:
                            self.get_logger().info('Timeout: nobody spoke.')
                            self.last_record_timed_out = True
                            return np.array([], dtype=np.float32)
                        continue

                    # Recording
                    phrase.append(chunk)
                    silence = 0 if is_voice else silence + 1
                    if silence > max_silence:
                        self.get_logger().info('End of the phrase.')
                        break
                    if len(phrase) > max_chunks:
                        self.get_logger().info('Longest phrase reached.')
                        break
        except Exception as e:
            self.get_logger().error(f'Microphone error: {e}')
            return np.array([], dtype=np.float32)

        return np.concatenate(phrase)

    def transcribe(self, audio):
        """Transcribe the audio with faster-whisper and publish it on /listened_text."""
        self.get_logger().info('Transcribing with faster-whisper...')
        segments, _ = self.asr.transcribe(
            audio,
            language=self.language,
            beam_size=3,
            temperature=0.0,
            condition_on_previous_text=False,
            no_speech_threshold=0.6,
            compression_ratio_threshold=2.4,
            log_prob_threshold=-1.0,
        )
        text = ''.join(segment.text for segment in segments).strip()
        self.get_logger().info(f'Result: "{text}"')
        self.pub.publish(String(data=text))
        return text

    def stt_callback(self, request, response):
        if not request.data:
            response.success = False
            response.message = "Send 'True' to start listening."
            return response
        with self.mic_lock:
            audio = self.record_audio_with_vad()
        if len(audio) == 0:
            response.success = False
            response.message = 'No audio detected or timeout.'
            return response
        try:
            response.message = self.transcribe(audio)
            response.success = True
        except Exception as e:
            self.get_logger().error(f'Inference error: {e}')
            response.success = False
            response.message = str(e)
        return response

    def listen(self, max_wait, should_stop, on_status):
        """Work of the /stt_action action (ListenActionServer takes the lock)."""
        audio = self.record_audio_with_vad(
            max_wait if max_wait > 0 else MAX_WAIT_SECONDS, should_stop, on_status)
        if should_stop():
            return False, False, '', 'canceled'
        if len(audio) == 0:
            return False, self.last_record_timed_out, '', 'No audio detected or timeout.'
        on_status('transcribing')
        try:
            text = self.transcribe(audio)
            return True, False, text, text
        except Exception as e:
            self.get_logger().error(f'Inference error: {e}')
            return False, False, '', str(e)


def main():
    rclpy.init()
    try:
        node = STTFasterWhisper()
        spin_multithreaded(node)
        node.destroy_node()
    except KeyboardInterrupt:  # Ctrl+C while the models are loading
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
