#!/usr/bin/env python3

# Copyright 2024 Antonio Bono
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
# limitations under the License.import openai

import os
import rclpy
import wave
import numpy as np
import sounddevice as sd
from rclpy.node import Node
from std_msgs.msg import String
import openai
import threading
import time

from simple_hri.stt_filter import is_hallucination
from simple_hri.vad_phrase import (
    ENDED, FRAME_SAMPLES, PhraseDetector, SAMPLE_RATE, STARTED, TOO_LONG, WAITING)
from simple_hri.voice_actions import ListenActionServer, spin_multithreaded

CHANNELS = 1
MAX_WAIT_SECONDS = 10.0  # Tiempo máximo esperando a que alguien empiece a hablar

class STTService(Node):
    def __init__(self):
        super().__init__("stt_service_node")

        self.declare_parameter('language', 'es')  # Language spoken: es, en...
        self.language = self.get_parameter('language').get_parameter_value().string_value
        # Discard what Whisper invents from noise (see stt_filter); false only to debug
        self.declare_parameter('hallucination_filter', True)
        self.hallucination_filter = self.get_parameter('hallucination_filter').value
        # Phrase detection (see vad_phrase)
        self.declare_parameter('vad_mode', 2)  # webrtcvad: 0 (takes noise for voice) to 3
        self.declare_parameter('noise_margin', 6.0)  # dB above the background that is voice
        self.declare_parameter('grace_period', 1.5)  # s of silence that end the phrase
        self.declare_parameter('max_phrase_secs', 60.0)  # s, longest phrase; 0 = no limit
        self.declare_parameter('pre_record', 0.5)  # s of audio kept before the voice

        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            self.get_logger().error("❌ No se encontró la clave de API de OpenAI. Exporta OPENAI_API_KEY antes de ejecutar.")
            exit(1)

        # Publisher
        self.pub = self.create_publisher(String, "/listened_text", 10)

        self.get_logger().info('✅ STTService con Whisper API inicializado')

        # Acción /stt_action. El cerrojo evita que dos objetivos usen el micrófono a la vez.
        self.mic_lock = threading.Lock()
        self.last_record_timed_out = False
        self.listen_action = ListenActionServer(self, self.listen, self.mic_lock)

    def record_audio_with_vad(self, max_wait=MAX_WAIT_SECONDS, should_stop=None, on_status=None):
        """
        Record one phrase (see vad_phrase): wait for voice (at most max_wait s), then record
        until grace_period s of silence or max_phrase_secs. Returns int16 audio, empty on
        timeout or cancellation. should_stop() is checked on every frame and on_status()
        reports the stage ("listening", "speech_detected"); both are optional.
        """
        p = self.get_parameter
        detector = PhraseDetector(
            vad_mode=p('vad_mode').value, noise_margin=p('noise_margin').value,
            grace_period=p('grace_period').value, max_phrase_secs=p('max_phrase_secs').value,
            pre_record=p('pre_record').value)
        start_wait_time = time.time()
        self.last_record_timed_out = False

        self.get_logger().info('🎙 Listening... (speak now)')
        if on_status:
            on_status('listening')

        try:
            with sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype='int16') as stream:
                while True:
                    if should_stop and should_stop():
                        self.get_logger().info('⏹ Recording canceled.')
                        return np.array([])

                    frame, overflow = stream.read(FRAME_SAMPLES)
                    if overflow:
                        self.get_logger().warning('⚠️ Audio overflow')
                    event = detector.add(frame)

                    if event == STARTED:
                        self.get_logger().info(
                            f'🔊 Voice detected (background {detector.background_db:.0f} dBFS), '
                            'recording...')
                        if on_status:
                            on_status('speech_detected')
                    elif event == WAITING and time.time() - start_wait_time > max_wait:
                        self.get_logger().info(
                            f'⏰ Timeout: nobody spoke (background {detector.background_db:.0f} '
                            'dBFS).')
                        self.last_record_timed_out = True
                        return np.array([])
                    elif event == ENDED:
                        self.get_logger().info('🛑 End of the phrase.')
                        break
                    elif event == TOO_LONG:
                        self.get_logger().info('🛑 Longest phrase reached.')
                        break

        except Exception as e:
            self.get_logger().error(f'❌ Microphone error: {e}')
            return np.array([])

        return detector.phrase()

    def transcribe(self, audio_data):
        """Transcribe el audio con la API de Whisper y publica el texto en /listened_text."""
        # Guardar en WAV temporal
        audio_path = "/tmp/audio.wav"
        with wave.open(audio_path, "wb") as wf:
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(2)
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(audio_data.tobytes())

        self.get_logger().info("🔍 Enviando audio a OpenAI Whisper API")

        with open(audio_path, "rb") as audio_file:
            client = openai.OpenAI(api_key=self.api_key)
            response = client.audio.transcriptions.create(
                model="whisper-1",
                file=audio_file,
                language=self.language,
            )

        transcribed_text = response.text.strip()
        # The API gives no no-speech probability: only the known hallucinations are discarded
        if self.hallucination_filter and is_hallucination(transcribed_text):
            self.get_logger().info(f'Descartado, no es voz: "{transcribed_text}"')
            return ''
        self.get_logger().info(f'📝 Transcripción: {transcribed_text}')

        # Publicar en el topic /listened_text
        msg = String()
        msg.data = transcribed_text
        self.pub.publish(msg)
        self.get_logger().info("📢 Texto publicado en /listened_text")
        return transcribed_text

    def listen(self, max_wait, should_stop, on_status):
        """Trabajo de la acción /stt_action (el cerrojo lo toma ListenActionServer)."""
        deadline = time.time() + (max_wait if max_wait > 0 else MAX_WAIT_SECONDS)
        while True:
            audio_data = self.record_audio_with_vad(
                deadline - time.time(), should_stop, on_status)
            if should_stop():
                return False, False, '', 'canceled'
            if len(audio_data) == 0:
                return False, self.last_record_timed_out, '', "No se detectó voz en la grabación."

            on_status('transcribing')
            try:
                text = self.transcribe(audio_data)
            except Exception as e:
                self.get_logger().error(f'❌ Error en el reconocimiento de voz: {e}')
                return False, False, '', str(e)
            if text:
                return True, False, text, text
            # Lo que oyó Whisper no era voz (ver stt_filter): seguir escuchando el resto de
            # max_wait, para que el ruido no termine la espera a la persona
            if time.time() >= deadline:
                return False, True, '', 'Nadie habló: lo que se oyó no era voz'


def main():
    rclpy.init()
    stt_service = STTService()
    try:
        spin_multithreaded(stt_service)
    finally:
        stt_service.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
