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

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python import get_package_share_directory
from huggingface_hub import InferenceClient

try:
    from simple_hri_interfaces.srv import YesNo
except ImportError:
    print("⚠️ WARNING: simple_hri_interfaces not found. Service will fail if called.")
    class YesNo:
        pass


class YesNoServiceLocal(Node):
    def __init__(self):
        super().__init__("yesno_service_node")

        self.declare_parameter("prompt_file", "yesno_prompt_es.txt")
        self.declare_parameter("model_id", "Qwen/Qwen2.5-Coder-32B-Instruct")
        self.declare_parameter("use_llm", False)  # By default, use simple pattern matching

        package_name = 'simple_hri'
        param_file_name = self.get_parameter("prompt_file").get_parameter_value().string_value
        self.use_llm = self.get_parameter("use_llm").get_parameter_value().bool_value

        if param_file_name.startswith("/"):
            self.prompt_path = param_file_name
        else:
            try:
                package_share_directory = get_package_share_directory(package_name)
                self.prompt_path = os.path.join(package_share_directory, "params", param_file_name)
            except Exception:
                self.get_logger().warn(f"⚠️ Could not find package share directory. Using relative path.")
                self.prompt_path = os.path.join("params", param_file_name)

        self.system_prompt = self._load_prompt_content()

        self.hf_token = os.getenv("HF_TOKEN")
        self.model_id = self.get_parameter("model_id").get_parameter_value().string_value

        # Only initialize HF client if LLM mode is enabled
        if self.use_llm:
            if not self.hf_token:
                self.get_logger().warn("⚠️ HF_TOKEN not found. LLM mode disabled, using pattern matching.")
                self.use_llm = False
                self.client = None
            else:
                try:
                    self.client = InferenceClient(api_key=self.hf_token)
                    self.get_logger().info(f'✅ HF Client initialized. Model: {self.model_id}')
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Failed to initialize HF client: {e}. Using pattern matching.")
                    self.use_llm = False
                    self.client = None
        else:
            self.client = None
            self.get_logger().info('✅ Using simple pattern matching for yes/no detection (LLM disabled)')

        if 'YesNo' in globals() and hasattr(YesNo, 'Request'):
            self.srv = self.create_service(YesNo, "yesno_service", self.yesno_callback)
            self.get_logger().info('✅ Service "yesno_service" ready.')
        else:
            self.get_logger().error("❌ Could not create service: Missing 'simple_hri_interfaces'")

        self.pub = self.create_publisher(String, "/yesno_result", 10)

    def _load_prompt_content(self) -> str:
        try:
            with open(self.prompt_path, 'r') as f:
                content = f.read()
            self.get_logger().info(f"📄 System Prompt loaded from: {self.prompt_path}")
            return content
        except FileNotFoundError:
            self.get_logger().warn(f"⚠️ Prompt file not found: {self.prompt_path}. Using default prompt.")
            return (
                "You are a yes/no detector assistant. Analyze the given text and respond with ONLY 'YES' or 'NO'. "
                "If the text contains affirmative words (yes, yeah, sure, correct, ok, affirmative, si, vale, etc.), respond 'YES'. "
                "If the text contains negative words (no, nope, negative, not, nunca, etc.), respond 'NO'. "
                "If uncertain, respond 'UNKNOWN'."
            )

    def extract_yesno_from_text(self, text: str) -> str:
        """Use Hugging Face to detect yes/no from text, with fallback to pattern matching."""
        
        # Use simple pattern matching if LLM is disabled or not available
        if not self.use_llm or not self.client:
            return self._simple_yesno_detection(text)

        # Try LLM approach first
        prompt_user = f"""Text: "{text}"
Response:"""

        try:
            messages = [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": prompt_user}
            ]
            
            completion = self.client.chat.completions.create(
                model=self.model_id,
                messages=messages,
                max_tokens=10,
                temperature=0.1
            )
            
            result = completion.choices[0].message.content.strip().upper()
            
            # Validate response
            if "YES" in result or "SÍ" in result or "SI" in result:
                return "YES"
            elif "NO" in result:
                return "NO"
            else:
                return "UNKNOWN"
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ LLM inference failed: {e}. Using fallback detection.")
            return self._simple_yesno_detection(text)

    def _simple_yesno_detection(self, text: str) -> str:
        """Fallback method using simple pattern matching."""
        text_lower = text.lower().strip()
        
        # Affirmative patterns (Spanish and English)
        yes_patterns = ['sí', 'si', 'yes', 'yeah', 'yep', 'sure', 'ok', 'okay', 
                       'vale', 'claro', 'correcto', 'afirmativo', 'efectivamente',
                       'por supuesto', 'desde luego']
        
        # Negative patterns (Spanish and English)
        no_patterns = ['no', 'nope', 'nunca', 'jamás', 'negativo', 'never',
                      'not', 'nah', 'tampoco']
        
        # Check for explicit matches
        for yes_word in yes_patterns:
            if yes_word in text_lower:
                return "YES"
        
        for no_word in no_patterns:
            if no_word in text_lower:
                return "NO"
        
        return "UNKNOWN"

    def yesno_callback(self, request, response):
        if not request.text:
            response.result = "ERROR: Empty text"
            return response

        text_to_process = request.text
        self.get_logger().info(f'🎯 Detecting yes/no in: "{text_to_process}"')
        
        result = self.extract_yesno_from_text(text_to_process)
        
        self.get_logger().info(f'✅ Result: {result}')

        # Publish to topic
        msg = String()
        msg.data = result
        self.pub.publish(msg)

        response.result = result
        return response


def main(args=None):
    rclpy.init(args=args)
    yesno_service = YesNoServiceLocal()
    
    try:
        rclpy.spin(yesno_service)
    except KeyboardInterrupt:
        pass
    finally:
        yesno_service.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
