#!/usr/bin/env python3

import os
import rclpy
import wave
import numpy as np
import sounddevice as sd
import webrtcvad
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
from simple_hri_interfaces.srv import YesNo  # Asegúrate de tener este servicio definido
from simple_hri import prompts
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

        self.declare_parameter("prompt_file", prompts.PROMPT_FILE)
        self.prompt = prompts.load_prompt(
            self.get_parameter("prompt_file").get_parameter_value().string_value, 'yesno')
        self.system_prompt = self.prompt.system
        self.get_logger().info(f'Prompt loaded from {self.prompt.path}')
        self.declare_parameter("model", "gpt-4.1-mini")
        self.model = self.get_parameter("model").get_parameter_value().string_value

        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            self.get_logger().error(
                "❌ No se encontró la clave de API de OpenAI. Exporta OPENAI_API_KEY antes de ejecutar."
            )
            exit(1)

        # Cliente de OpenAI reutilizable
        self.client = OpenAI(api_key=api_key)


        self.srv = self.create_service(YesNo, "yesno_service", self.yesno_callback)
        self.pub = self.create_publisher(String, "/yesno_result", 10)

        self.get_logger().info('✅ YesNoService inicializado')
    
    def extract_yesno_from_text(self,text: str) -> str:
        
        prompt_user = self.prompt.user(text=text)

        response = self.client.responses.create(
            model=self.model,  # gpt-4.1-mini by default: fast and cheap
            input=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": prompt_user},
            ],
        )

        # YES, NO or UNKNOWN, as the other Yes/No backends
        return prompts.parse_yesno(response.output_text)

    def yesno_callback(self, sRequest, sResponse):

        try:
            self.text = sRequest.text

            resp = self.extract_yesno_from_text(self.text)
            self.get_logger().info(f'✅ Extraído: {resp}')

            # Publish on /yesno_result, as the other Yes/No backends
            msg = String()
            msg.data = resp
            self.pub.publish(msg)

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