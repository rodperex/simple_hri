#!/usr/bin/env python3

# Copyright 2025 Rodrigo Pérez-Rodríguez
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

import rclpy
import numpy as np
import sounddevice as sd
import webrtcvad
import time
import whisper
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import String
from scipy.io.wavfile import write

# Parámetros
SAMPLE_RATE = 16000
FRAME_DURATION = 30  # ms
CHANNELS = 1
VAD_SENSITIVITY = 0  # 0=permisivo, 3=estricto
SILENCE_DURATION = 1.5  # Segundos de silencio para cortar la grabación

MODEL_NAME = "small"
whisper_model = whisper.load_model(MODEL_NAME)

class STTService(Node):
    def __init__(self):
        super().__init__("stt_service_node")
        # Servicio
        self.srv = self.create_service(SetBool, "stt_service", self.stt_callback)
        # Publisher
        self.pub = self.create_publisher(String, "/listened_text", 10)
        self.get_logger().info(f'✅ STTService con Whisper ({MODEL_NAME}) inicializado. VAD siempre activo.')

        # Inicializar VAD
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(VAD_SENSITIVITY)

    def record_audio_with_vad(self):
        frame_length = int(SAMPLE_RATE * FRAME_DURATION / 1000)
        audio_buffer = []
        last_voice_time = None

        self.get_logger().info('🎙 Esperando detección de voz...')

        try:
            with sd.InputStream(samplerate=SAMPLE_RATE, channels=CHANNELS, dtype='int16') as stream:
                while True:
                    frame, overflow = stream.read(frame_length)
                    frame = frame.flatten()
                    frame_bytes = frame.tobytes()

                    is_speech = self.vad.is_speech(frame_bytes, SAMPLE_RATE)

                    if is_speech:
                        last_voice_time = time.time()
                        audio_buffer.append(frame)
                        self.get_logger().info('🔊 Voz detectada...')
                    elif last_voice_time is not None and time.time() - last_voice_time > SILENCE_DURATION:
                        self.get_logger().info('🛑 Silencio prolongado, terminando grabación.')
                        break
        except Exception as e:
            self.get_logger().error(f"❌ Error al abrir micrófono: {e}")
            return np.array([])

        if audio_buffer:
            audio_data = np.concatenate(audio_buffer)
            write("/tmp/audio.wav", SAMPLE_RATE, audio_data)
            self.get_logger().info(f"✅ Grabación guardada en /tmp/audio.wav, {audio_data.shape[0]} samples")
            return audio_data
        else:
            self.get_logger().info("⚠️ No se detectó voz.")
            return np.array([])

    def stt_callback(self, sRequest, sResponse):
        if not sRequest.data:
            sResponse.success = False
            sResponse.message = "Llama al servicio con 'True' para iniciar reconocimiento."
            return sResponse

        try:
            audio_data = self.record_audio_with_vad()
            if len(audio_data) == 0:
                sResponse.success = True
                sResponse.message = "No se detectó voz."
                return sResponse

            # Convertir a float32 para Whisper
            audio_float = audio_data.astype(np.float32) / 32768.0
            self.get_logger().info("🔍 Procesando audio con Whisper...")
            result = whisper_model.transcribe(audio_float, language="es", fp16=False)

            transcribed_text = result["text"]
            self.get_logger().info(f'📝 Transcripción: {transcribed_text}')

            # Publicar en el topic /listened_text
            msg = String()
            msg.data = transcribed_text
            self.pub.publish(msg)
            self.get_logger().info("📢 Texto publicado en /listened_text")

            # Responder al servicio también
            sResponse.success = True
            sResponse.message = transcribed_text

        except Exception as e:
            self.get_logger().error(f'❌ Error en STT: {e}')
            sResponse.success = False
            sResponse.message = str(e)

        return sResponse

def main():
    rclpy.init()
    stt_service = STTService()
    rclpy.spin(stt_service)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
