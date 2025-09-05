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
from rclpy.node import Node
from std_srvs.srv import SetBool
from hni_interfaces.srv import TextToSpeech

class TestServices(Node):
    def __init__(self):
        super().__init__('stt_to_tts_client')

        # Cliente del servicio STT
        self.stt_client = self.create_client(SetBool, '/stt_service')
        while not self.stt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/stt_service no disponible, esperando...')

        # Cliente del servicio TTS
        self.tts_client = self.create_client(TextToSpeech, '/tts_service')
        while not self.tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/tts_service no disponible, esperando...')

        self.get_logger().info("✅ Clientes STT y TTS listos para usar.")

    def run(self):
        # 1️⃣ Llamar al servicio STT
        self.get_logger().info("🎤 Iniciando reconocimiento de voz (STT)...")
        stt_req = SetBool.Request()
        stt_req.data = True  # Indica al servicio que inicie grabación

        stt_future = self.stt_client.call_async(stt_req)
        rclpy.spin_until_future_complete(self, stt_future)
        stt_response = stt_future.result()

        if not stt_response.success:
            self.get_logger().error(f"❌ Error en STT: {stt_response.message}")
            return

        transcribed_text = stt_response.message
        self.get_logger().info(f"📝 Transcripción obtenida: {transcribed_text}")

        # 2️⃣ Llamar al servicio TTS para reproducir la transcripción
        self.get_logger().info("🔊 Enviando texto a TTS para reproducción...")
        tts_req = TextToSpeech.Request()
        tts_req.text = transcribed_text

        tts_future = self.tts_client.call_async(tts_req)
        rclpy.spin_until_future_complete(self, tts_future)
        tts_response = tts_future.result()

        if tts_response.success:
            self.get_logger().info("✅ TTS ejecutado correctamente")
        else:
            self.get_logger().error(f"❌ Error en TTS: {tts_response.debug}")

def main(args=None):
    rclpy.init(args=args)
    node = TestServices()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
