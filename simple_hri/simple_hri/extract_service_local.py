#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python import get_package_share_directory
from huggingface_hub import InferenceClient

try:
    from simple_hri_interfaces.srv import Extract
except ImportError:
    print("⚠️ ADVERTENCIA: simple_hri_interfaces no encontrado. El servicio fallará si se llama.")
    class Extract:
        pass

class ExtractService(Node):
    def __init__(self):
        super().__init__("extract_service_node")

        self.declare_parameter("prompt_file", "basic_prompt_es.txt")
        self.declare_parameter("model_id", "Qwen/Qwen2.5-Coder-32B-Instruct")
        self.declare_parameter("use_llm", False)  # By default, use simple keyword extraction

        package_name = 'simple_hri'
        param_file_name = self.get_parameter("prompt_file").get_parameter_value().string_value
        self.use_llm = self.get_parameter("use_llm").get_parameter_value().bool_value

        if param_file_name.startswith("/"):
            self.prompt_path = param_file_name
        else:
            try:
                pkg_share = get_package_share_directory(package_name)
                self.prompt_path = os.path.join(pkg_share, "params", param_file_name)
            except Exception:
                self.get_logger().warn(f"Paquete '{package_name}' no encontrado. Usando ruta local relativa.")
                self.prompt_path = os.path.join("params", param_file_name)

        self.system_prompt = self._load_prompt_content()

        self.hf_token = os.getenv("HF_TOKEN")
        self.model_id = self.get_parameter("model_id").get_parameter_value().string_value

        # Only initialize HF client if LLM mode is enabled
        if self.use_llm:
            if not self.hf_token:
                self.get_logger().warn("⚠️ HF_TOKEN not found. LLM mode disabled, using keyword extraction.")
                self.use_llm = False
                self.client = None
            else:
                try:
                    self.client = InferenceClient(api_key=self.hf_token)
                    self.get_logger().info(f'✅ HF Client initialized. Model: {self.model_id}')
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Failed to initialize HF client: {e}. Using keyword extraction.")
                    self.use_llm = False
                    self.client = None
        else:
            self.client = None
            self.get_logger().info('✅ Using simple keyword extraction (LLM disabled)')

        self.interest = ""
        if 'Extract' in globals() and hasattr(Extract, 'Request'):
            self.srv = self.create_service(Extract, "extract_service", self.extract_callback)
            self.get_logger().info('✅ Servicio "extract_service" listo.')
        else:
            self.get_logger().error("❌ No se pudo crear el servicio: Falta 'simple_hri_interfaces'")

        self.pub = self.create_publisher(String, "/extracted_text", 10)

    def _load_prompt_content(self) -> str:
        try:
            with open(self.prompt_path, 'r') as f:
                content = f.read()
            self.get_logger().info(f"📄 System Prompt cargado desde: {self.prompt_path}")
            return content
        except FileNotFoundError:
            self.get_logger().warn(f"⚠️ No se encontró {self.prompt_path}. Usando prompt por defecto.")
            return (
                "Eres un asistente extractor. Extrae SOLO la información solicitada en 'interest' "
                "del texto dado. Si hay varios, sepáralos con ';'. Si no hay nada, responde 'NONE'."
            )

    def extract_interest_from_text(self, interest: str, text: str) -> str:
        """Use Hugging Face to extract information, with fallback to keyword extraction."""
        
        # Use simple keyword extraction if LLM is disabled or not available
        if not self.use_llm or not self.client:
            return self._simple_keyword_extraction(interest, text)

        # Try LLM approach first
        prompt_user = f"""interest: "{interest}"
            text: "{text}"
            response:"""

        try:
            response = self.client.chat_completion(
                model=self.model_id,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": prompt_user},
                ],
                max_tokens=150,     
                temperature=0.1,
                stream=False
            )

            extracted = response.choices[0].message.content.strip()
            
            # Basic cleanup in case model is verbose
            if "Here is the" in extracted or "Respuesta:" in extracted:
                extracted = extracted.split(":")[-1].strip()
                
            return extracted
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ LLM extraction failed: {e}. Using keyword fallback.")
            return self._simple_keyword_extraction(interest, text)

    def _simple_keyword_extraction(self, interest: str, text: str) -> str:
        """Fallback method using simple keyword matching and context extraction."""
        text_lower = text.lower()
        interest_lower = interest.lower()
        
        # If text is empty
        if not text.strip():
            return "NONE"
        
        # Split text into words/tokens
        words = text.split()
        
        # Look for the interest keyword in the text
        results = []
        
        # Strategy 1: Find sentences/phrases containing the interest keyword
        sentences = text.replace('?', '.').replace('!', '.').split('.')
        for sentence in sentences:
            if interest_lower in sentence.lower():
                # Extract words around the interest
                words_in_sentence = sentence.strip().split()
                results.extend(words_in_sentence)
        
        # Strategy 2: If no direct match, look for contextual words
        if not results:
            # Common food/drink keywords (expand based on your domain)
            food_drinks = ['pizza', 'burger', 'sandwich', 'taco', 'pasta', 'salad',
                          'water', 'juice', 'coffee', 'tea', 'beer', 'wine', 'soda',
                          'coca', 'pepsi', 'fanta', 'sprite', 'agua', 'zumo', 'café',
                          'cerveza', 'vino', 'refresco', 'margarita', 'napolitana']
            
            for word in words:
                word_clean = word.lower().strip(',.!?;')
                if any(keyword in word_clean for keyword in food_drinks):
                    results.append(word.strip(',.!?;'))
        
        # Strategy 3: If still nothing, return the full text (might contain what we need)
        if not results:
            if len(words) <= 10:
                return text.strip()
            else:
                return "NONE"
        
        # Remove duplicates and join
        unique_results = list(dict.fromkeys(results))  # Preserve order
        return '; '.join(unique_results[:5]) if unique_results else "NONE"  # Limit to 5 items

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