#!/usr/bin/env python3

import base64
import io
import os
import traceback
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage
from ament_index_python import get_package_share_directory
from huggingface_hub import InferenceClient
from PIL import Image as PILImage

try:
    from simple_hri_interfaces.srv import AnalyzeImage
except ImportError:
    print("ADVERTENCIA: simple_hri_interfaces no encontrado. El servicio fallara si se llama.")
    class AnalyzeImage:
        pass


class AnalyzeImageService(Node):
    def __init__(self):
        super().__init__("analyze_image_service_node")

        self.declare_parameter("prompt_file", "basic_visual_prompt_es.txt")
        self.declare_parameter("model_id", "Qwen/Qwen3-VL-30B-A3B-Instruct")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("provider", "novita")

        self.model_id = self.get_parameter("model_id").get_parameter_value().string_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.provider = self.get_parameter("provider").get_parameter_value().string_value
        self.prompt_path = self._resolve_prompt_path(
            self.get_parameter("prompt_file").get_parameter_value().string_value)
        self.system_prompt = self._load_prompt_content()

        self.last_image: Optional[RosImage] = None
        self.image_sub = self.create_subscription(
            RosImage,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data)
        self.get_logger().info(f'Suscrito a imagenes en: {self.image_topic}')

        self.hf_token = os.getenv("HF_TOKEN")
        if self.hf_token:
            self.client = InferenceClient(provider=self.provider, api_key=self.hf_token)
            self.get_logger().info(f'Cliente HF inicializado. Modelo: {self.model_id}. Provider: {self.provider}')
        else:
            self.client = None
            self.get_logger().error("Falta HF_TOKEN. Ejecuta: export HF_TOKEN='tu_token'")

        if 'AnalyzeImage' in globals() and hasattr(AnalyzeImage, 'Request'):
            self.srv = self.create_service(
                AnalyzeImage,
                "analyze_image_service",
                self.analyze_callback)
            self.get_logger().info('Servicio "analyze_image_service" listo.')
        else:
            self.get_logger().error("No se pudo crear el servicio: falta AnalyzeImage.srv generado")

        self.pub = self.create_publisher(String, "/analyzed_image_text", 10)

    def _resolve_prompt_path(self, prompt_file: str) -> str:
        if prompt_file.startswith("/"):
            return prompt_file
        try:
            pkg_share = get_package_share_directory('simple_hri')
            return os.path.join(pkg_share, "params", prompt_file)
        except Exception:
            return os.path.join("params", prompt_file)

    def _load_prompt_content(self) -> str:
        try:
            with open(self.prompt_path, 'r') as f:
                content = f.read()
            self.get_logger().info(f"System prompt cargado desde: {self.prompt_path}")
            return content
        except FileNotFoundError:
            self.get_logger().warn(f"No se encontro {self.prompt_path}. Usando prompt por defecto.")
            return (
                "Estás trabajando como asistente para una persona ciega. Haz lo que te pida."

            )

    def image_callback(self, msg: RosImage):
        self.last_image = msg

    def _ros_image_to_data_url(self, msg: RosImage) -> str:
        encoding = msg.encoding.lower()
        width = msg.width
        height = msg.height
        data = bytes(msg.data)

        if encoding in ("rgb8", "bgr8"):
            mode = "RGB"
            expected_step = width * 3
            rows = []
            for y in range(height):
                row = bytearray(data[y * msg.step:y * msg.step + expected_step])
                if encoding == "bgr8":
                    for i in range(0, len(row), 3):
                        row[i], row[i + 2] = row[i + 2], row[i]
                rows.append(bytes(row))
            image = PILImage.frombytes(mode, (width, height), b"".join(rows))
        elif encoding in ("mono8", "8uc1"):
            rows = [data[y * msg.step:y * msg.step + width] for y in range(height)]
            image = PILImage.frombytes("L", (width, height), b"".join(rows)).convert("RGB")
        elif encoding in ("rgba8", "bgra8"):
            expected_step = width * 4
            rows = []
            for y in range(height):
                row = bytearray(data[y * msg.step:y * msg.step + expected_step])
                if encoding == "bgra8":
                    for i in range(0, len(row), 4):
                        row[i], row[i + 2] = row[i + 2], row[i]
                rows.append(bytes(row))
            image = PILImage.frombytes("RGBA", (width, height), b"".join(rows)).convert("RGB")
        else:
            raise ValueError(f"Encoding de imagen no soportado: {msg.encoding}")

        buffer = io.BytesIO()
        image.save(buffer, format="JPEG", quality=85)
        encoded = base64.b64encode(buffer.getvalue()).decode("ascii")
        return f"data:image/jpeg;base64,{encoded}"

    def analyze_image(self, prompt: str) -> str:
        if not self.client:
            return "ERROR_NO_HF_TOKEN"
        if self.last_image is None:
            return "ERROR_NO_IMAGE"

        image_url = self._ros_image_to_data_url(self.last_image)
        user_prompt = prompt.strip() if prompt.strip() else self.system_prompt

        try:
            response = self.client.chat_completion(
                model=self.model_id,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": user_prompt},
                            {"type": "image_url", "image_url": {"url": image_url}},
                        ],
                    },
                ],
                max_tokens=250,
                temperature=0.1,
                stream=False)
            return response.choices[0].message.content.strip()
        except Exception as e:
            self.get_logger().error(f"Error llamando a Hugging Face Vision API: {type(e).__name__}: {repr(e)}")
            self.get_logger().debug(traceback.format_exc())
            return f"ERROR_API: {type(e).__name__}: {e}"

    def analyze_callback(self, request, response):
        result = self.analyze_image(request.prompt)
        self.get_logger().info(f"Resultado analisis imagen: {result}")
        msg = String()
        msg.data = result
        self.pub.publish(msg)
        response.result = result
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AnalyzeImageService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
