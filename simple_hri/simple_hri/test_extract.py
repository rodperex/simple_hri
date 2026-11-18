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
from simple_hri_interfaces.srv import Extract


class TestExtract(Node):
    def __init__(self):
        super().__init__('extract_client')

        # Cliente del servicio Extract
        self.extract_client = self.create_client(Extract, '/extract_service')
        while not self.extract_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/extract_service no disponible, esperando...')

        self.get_logger().info("✅ Cliente Extract listo para usar.")

    def run(self):

        self.get_logger().info("🔍 Enviando texto al servicio Extract...")
        extract_req = Extract.Request()
        transcribed_text = "Quiero una cocacola, una cerveza y una pizza margarita."
        extract_req.text = transcribed_text
        # extract_req.interes = "bebida"
        extract_req.interest = "plato principal"  # Ejemplo de interés

        extract_future = self.extract_client.call_async(extract_req)
        rclpy.spin_until_future_complete(self, extract_future)
        extract_response = extract_future.result()

        if extract_response.result == "ERROR":
            self.get_logger().error("❌ Error en Extract")
            return

        transcribed_text = extract_response.result
        self.get_logger().info(f"📝 Transcripción obtenida: {transcribed_text}")


def main(args=None):
    rclpy.init(args=args)
    node = TestExtract()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
