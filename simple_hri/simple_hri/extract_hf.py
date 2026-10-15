#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from huggingface_hub import InferenceClient

from simple_hri import prompts

try:
    from simple_hri_interfaces.srv import Extract
except ImportError:
    print("⚠️ ADVERTENCIA: simple_hri_interfaces no encontrado. El servicio fallará si se llama.")
    class Extract:
        pass

class ExtractService(Node):
    def __init__(self):
        super().__init__("extract_service_node")

        self.declare_parameter("prompt_file", prompts.PROMPT_FILE)
        self.declare_parameter("model_id", "meta-llama/Meta-Llama-3-8B-Instruct")

        self.prompt = prompts.load_prompt(
            self.get_parameter("prompt_file").get_parameter_value().string_value, 'extract')
        self.system_prompt = self.prompt.system
        self.get_logger().info(f"📄 Prompt loaded from {self.prompt.path}")

        self.hf_token = os.getenv("HF_TOKEN")

        if not self.hf_token:
            self.get_logger().fatal("❌ CRITICAL: Falta variable HF_TOKEN. Ejecuta: export HF_TOKEN='tu_token'")
        
        self.model_id = self.get_parameter("model_id").get_parameter_value().string_value
        
        if self.hf_token:
            self.get_logger().info(f'🚀 Inicializando cliente Hugging Face para modelo: {self.model_id}...')
            self.client = InferenceClient(api_key=self.hf_token)
            self.get_logger().info(f'✅ Cliente HF inicializado. Modelo: {self.model_id}')
        else:
            self.client = None

        self.interest = ""
        if 'Extract' in globals() and hasattr(Extract, 'Request'):
            self.srv = self.create_service(Extract, "extract_service", self.extract_callback)
            self.get_logger().info('✅ Servicio "extract_service" listo.')
        else:
            self.get_logger().error("❌ No se pudo crear el servicio: Falta 'simple_hri_interfaces'")

        self.pub = self.create_publisher(String, "/extracted_text", 10)

    def extract_interest_from_text(self, interest: str, text: str) -> str:
        """Usa Hugging Face para extraer información."""
        if not self.client:
            return "ERROR_NO_TOKEN"

        prompt_user = self.prompt.user(interest=interest, text=text)

        try:
            response = self.client.chat_completion(
                model=self.model_id,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": prompt_user},
                ],
                max_tokens=150,     
                temperature=0.1,    # Un poco de temp ayuda a evitar bucles, pero bajo para precisión
                stream=False
            )

            return prompts.clean_extracted(response.choices[0].message.content)
            
        except Exception as e:
            self.get_logger().error(f"Error llamando a Hugging Face API: {e}")
            return "ERROR_API"

    def extract_callback(self, request, response):
        if not request.interest:
            response.result = "ERROR: Missing interest"
            return response

        text_to_process = request.text if request.text else ""
        if not text_to_process:
            response.result = "NONE"
            return response

        self.get_logger().info(f'🎯 Buscando "{request.interest}" en el texto...')
        
        extracted_text = self.extract_interest_from_text(request.interest, text_to_process)
        
        self.get_logger().info(f'✅ Resultado: {extracted_text}')

        msg = String()
        msg.data = extracted_text
        self.pub.publish(msg)

        response.result = extracted_text
        return response

def main(args=None):
    rclpy.init(args=args)
    extract_service = ExtractService()
    
    try:
        rclpy.spin(extract_service)
    except KeyboardInterrupt:
        pass
    finally:
        extract_service.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()