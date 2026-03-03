#!/usr/bin/env python3

import os

# Configurar caché de transformers ANTES de importar
cache_dir = os.path.abspath(os.path.join(os.getcwd(), 'models'))
os.makedirs(cache_dir, exist_ok=True)
os.environ['HF_HOME'] = cache_dir

import torch
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python import get_package_share_directory
from transformers import pipeline

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
        # Modelos disponibles (ordenados por tamaño):
        # "google/flan-t5-small"              # ~300MB, más rápido (DEFAULT)
        # "google/flan-t5-base"               # ~900MB, mejor calidad
        # "microsoft/Phi-3-mini-4k-instruct"  # ~4GB, requiere más RAM
        # "Qwen/Qwen2.5-Coder-3B-Instruct"    # ~6GB, muy bueno pero pesado
        self.declare_parameter("model_id", "google/flan-t5-small")
        self.declare_parameter("use_gpu", False)

        package_name = 'simple_hri'
        param_file_name = self.get_parameter("prompt_file").get_parameter_value().string_value

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
        self.model_id = self.get_parameter("model_id").get_parameter_value().string_value
        use_gpu = self.get_parameter("use_gpu").get_parameter_value().bool_value

        # Device selection
        self.device = -1  # CPU
        if use_gpu and torch.cuda.is_available():
            self.device = 0  # First GPU
            self.get_logger().info("🚀 CUDA detected. Using GPU for inference.")
        elif use_gpu:
            self.get_logger().warn("⚠️ GPU requested but CUDA not available. Falling back to CPU.")

        # Cargar modelo local
        self.get_logger().info(f'⏳ Cargando modelo local: {self.model_id}...')
        try:
            self.generator = pipeline(
                "text2text-generation",
                model=self.model_id,
                device=self.device,
                max_length=150
            )
            self.get_logger().info(f'✅ Modelo {self.model_id} cargado exitosamente.')
        except Exception as e:
            self.get_logger().error(f"❌ Error cargando modelo: {e}")
            self.generator = None

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
        """Usa modelo local para extraer información."""
        if not self.generator:
            return "ERROR_NO_MODEL"

        # Crear prompt simplificado para modelos T5
        prompt = f"{self.system_prompt}\n\nInterés: {interest}\nTexto: {text}\nRespuesta:"

        try:
            result = self.generator(
                prompt,
                max_length=150,
                temperature=0.1,
                do_sample=True,
                top_p=0.9
            )

            extracted = result[0]['generated_text'].strip()
            
            # Limpieza básica
            if "Here is the" in extracted or "Respuesta:" in extracted:
                extracted = extracted.split(":")[-1].strip()
            
            # Si está vacío o muy corto, devolver NONE
            if len(extracted) < 2:
                extracted = "NONE"
                
            return extracted
            
        except Exception as e:
            self.get_logger().error(f"Error en inferencia local: {e}")
            return "ERROR_INFERENCE"

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