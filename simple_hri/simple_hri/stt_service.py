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
import webrtcvad
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import String
import openai
import threading
import time

from simple_hri.voice_actions import ListenActionServer, spin_multithreaded

# Parámetros
SAMPLE_RATE = 16000
FRAME_DURATION = 30
CHANNELS = 1
VAD_SENSITIVITY = 2  # 0=muy permisivo (ruido = voz, nunca corta), 3=muy estricto
SILENCE_DURATION = 1.5
MAX_WAIT_SECONDS = 10.0  # Tiempo máximo esperando a que alguien empiece a hablar

class STTService(Node):
    def __init__(self):
        super().__init__("stt_service_node")

        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            self.get_logger().error("❌ No se encontró la clave de API de OpenAI. Exporta OPENAI_API_KEY antes de ejecutar.")
            exit(1)

        # Servicio
        self.srv = self.create_service(SetBool, "stt_service", self.stt_callback)
        # Publisher
        self.pub = self.create_publisher(String, "/listened_text", 10)

        self.get_logger().info('✅ STTService con Whisper API inicializado')

        # Inicializar el detector de actividad de voz (VAD)
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(VAD_SENSITIVITY)

        # Acción /stt_action: mismo trabajo que el servicio, con feedback y cancelación.
        # El cerrojo evita que servicio y acción usen el micrófono a la vez.
        self.mic_lock = threading.Lock()
        self.last_record_timed_out = False
        self.listen_action = ListenActionServer(self, self.listen, self.mic_lock)

    def record_audio_with_vad(self, max_wait=MAX_WAIT_SECONDS, should_stop=None, on_status=None):
        # should_stop() se consulta en cada fragmento para poder cancelar, y on_status()
        # informa de la fase ("listening", "speech_detected"). Ambos son opcionales.
        self.get_logger().info('🎙 Esperando detección de voz...')
        if on_status:
            on_status('listening')

        audio_buffer = []
        last_voice_time = None
        start_wait_time = time.time()
        self.last_record_timed_out = False

        stream = sd.InputStream(samplerate=SAMPLE_RATE, channels=CHANNELS, dtype=np.int16)
        with stream:
            while True:
                if should_stop and should_stop():
                    self.get_logger().info('⏹ Grabación cancelada.')
                    return np.array([])

                frame, _ = stream.read(int(SAMPLE_RATE * FRAME_DURATION / 1000))
                frame_bytes = frame.tobytes()

                is_speech = self.vad.is_speech(frame_bytes, SAMPLE_RATE)

                if is_speech:
                    if last_voice_time is None:
                        self.get_logger().info('🔊 ¡Detección de voz iniciada! Comenzando grabación...')
                        if on_status:
                            on_status('speech_detected')
                    last_voice_time = time.time()
                    audio_buffer.append(frame)
                elif last_voice_time is not None and time.time() - last_voice_time > SILENCE_DURATION:
                    self.get_logger().info('🛑 Se detectó silencio prolongado. Terminando grabación.')
                    break
                elif last_voice_time is None and time.time() - start_wait_time > max_wait:
                    self.get_logger().info('⏰ Timeout: no se detectó voz.')
                    self.last_record_timed_out = True
                    break

        if audio_buffer:
            audio_data = np.concatenate(audio_buffer, axis=0)
            return audio_data
        else:
            return np.array([])

    def stt_callback(self, sRequest, sResponse):
        if not sRequest.data:
            sResponse.success = False
            sResponse.message = "El servicio debe llamarse con 'True' para iniciar reconocimiento."
            return sResponse

        try:
            with self.mic_lock:
                audio_data = self.record_audio_with_vad()
            if len(audio_data) == 0:
                self.get_logger().info("⚠️ No se detectó voz.")
                sResponse.success = True
                sResponse.message = "No se detectó voz en la grabación."
                return sResponse

            transcribed_text = self.transcribe(audio_data)

            # Responder al servicio
            sResponse.success = True
            sResponse.message = transcribed_text

        except Exception as e:
            self.get_logger().error(f'❌ Error en el reconocimiento de voz: {e}')
            sResponse.success = False
            sResponse.message = str(e)

        return sResponse

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
                language="es",
            )

        transcribed_text = response.text
        self.get_logger().info(f'📝 Transcripción: {transcribed_text}')

        # Publicar en el topic /listened_text
        msg = String()
        msg.data = transcribed_text
        self.pub.publish(msg)
        self.get_logger().info("📢 Texto publicado en /listened_text")
        return transcribed_text

    def listen(self, max_wait, should_stop, on_status):
        """Trabajo de la acción /stt_action (el cerrojo lo toma ListenActionServer)."""
        audio_data = self.record_audio_with_vad(
            max_wait if max_wait > 0 else MAX_WAIT_SECONDS, should_stop, on_status)

        if should_stop():
            return False, False, '', 'canceled'
        if len(audio_data) == 0:
            return False, self.last_record_timed_out, '', "No se detectó voz en la grabación."

        on_status('transcribing')
        try:
            text = self.transcribe(audio_data)
            return True, False, text, text
        except Exception as e:
            self.get_logger().error(f'❌ Error en el reconocimiento de voz: {e}')
            return False, False, '', str(e)


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
