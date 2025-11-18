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
from simple_hri_interfaces.srv import Extract  # Asegúrate de tener este servicio definido
import time

# Parámetros
SAMPLE_RATE = 16000
FRAME_DURATION = 30
CHANNELS = 1
VAD_SENSITIVITY = 0
SILENCE_DURATION = 1.5


class ExtractService(Node):
    def __init__(self):
        super().__init__("extract_service_node")

        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            self.get_logger().error(
                "❌ No se encontró la clave de API de OpenAI. Exporta OPENAI_API_KEY antes de ejecutar."
            )
            exit(1)

        # Cliente de OpenAI reutilizable
        self.client = OpenAI(api_key=api_key)

        # "interest" vendrá en la petición del servicio
        self.interest = ""

        self.srv = self.create_service(Extract, "extract_service", self.extract_callback)

        # Publisher: texto extraído
        self.pub = self.create_publisher(String, "/extracted_text", 10)

        self.get_logger().info('✅ ExtractService inicializado')
    
    def extract_interest_from_text(self, interest: str, text: str) -> str:
        """Usa un modelo de lenguaje para extraer lo relevante según el 'interest'."""

        prompt_system = (
            "Eres un extractor de información.\n\n"
            "Recibes una categoría de interés ('interest') y una frase del usuario.\n"
            "Tu tarea es extraer SOLO la palabra o frase más relevante relacionada con "
            "esa categoría.\n"
            "No añadas explicaciones ni texto extra.\n"
            "Si no hay nada relevante devuelve exactamente: NINGUNO."
        )

        prompt_user = f"""interest: "{interest}"
                        texto: "{text}"
                        respuesta:"""

        response = self.client.responses.create(
            model="gpt-4.1-mini",  # modelo rápido y barato
            input=[
                {"role": "system", "content": prompt_system},
                {"role": "user", "content": prompt_user},
            ],
        )

        extracted = response.output_text.strip()
        return extracted

    def extract_callback(self, sRequest, sResponse):
        if not sRequest.interest:
            sResponse.result = False
            sResponse.message = "El campo 'interest' debe contener el interés (ej. 'bebida')."
            return sResponse

        try:
            self.interest = sRequest.interest
            self.text = sRequest.text

            self.get_logger().info(f'🎯 Extrayendo texto relevante para interés: {self.interest}')
            extracted_text = self.extract_interest_from_text(self.interest, self.text)
            self.get_logger().info(f'✅ Extraído: {extracted_text}')

            # Publicar en el topic /extracted_text
            msg = String()
            msg.data = extracted_text
            self.pub.publish(msg)
            self.get_logger().info("📢 Texto extraído publicado en /extracted_text")

            # Responder al servicio con el texto extraído
            sResponse.result = extracted_text

        except Exception as e:
            self.get_logger().error(f'❌ Error en el reconocimiento o extracción: {e}')
            sResponse.result = "ERROR"
            sResponse.message = str(e)

        return sResponse


def main():
    rclpy.init()
    extract_service = ExtractService()
    rclpy.spin(extract_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()