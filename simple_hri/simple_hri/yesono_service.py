#!/usr/bin/env python3

import os
from ament_index_python import get_package_share_directory
import rclpy
import wave
import numpy as np
import sounddevice as sd
import webrtcvad
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
from simple_hri_interfaces.srv import YesNo  # Asegúrate de tener este servicio definido
import time

# Parámetros
SAMPLE_RATE = 16000
FRAME_DURATION = 30
CHANNELS = 1
VAD_SENSITIVITY = 0
SILENCE_DURATION = 1.5


class YesNoService(Node):
    def __init__(self):
        super().__init__("yesno_service_node")

        package_name = 'simple_hri'
        package_share_directory = get_package_share_directory(package_name)
        
        self.declare_parameter("prompt_file", "yesno_prompt_es.txt")

        self.prompt_file = self.base_frame = self.get_parameter("prompt_file").get_parameter_value().string_value
        self.prompt_file = os.path.join(package_share_directory + "/params/", self.prompt_file)

        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            self.get_logger().error(
                "❌ No se encontró la clave de API de OpenAI. Exporta OPENAI_API_KEY antes de ejecutar."
            )
            exit(1)

        # Cliente de OpenAI reutilizable
        self.client = OpenAI(api_key=api_key)


        self.srv = self.create_service(YesNo, "yesno_service", self.yesno_callback)

        self.get_logger().info('✅ YesNoService inicializado')
    
    def extract_yesno_from_text(self,text: str) -> str:
        
        with open(self.prompt_file, 'r') as f:
            prompt_system = f.read()
    

        prompt_user = f"""Texto: {text}"""

        response = self.client.responses.create(
            model="gpt-4.1-mini",  # modelo rápido y barato
            input=[
                {"role": "system", "content": prompt_system},
                {"role": "user", "content": prompt_user},
            ],
        )

        extracted = response.output_text.strip()
        return extracted

    def yesno_callback(self, sRequest, sResponse):

        try:
            self.text = sRequest.text

            resp = self.extract_yesno_from_text(self.text)
            self.get_logger().info(f'✅ Extraído: {resp}')

            # Publicar en el topic /extracted_text
            msg = String()
            msg.data = resp

            # Responder al servicio con el texto extraído
            sResponse.result = resp

        except Exception as e:
            self.get_logger().error(f'❌ Error en el reconocimiento o extracción: {e}')
            sResponse.result = "ERROR:" + str(e)
        return sResponse


def main():
    rclpy.init()
    yesno_service = YesNoService()
    rclpy.spin(yesno_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()