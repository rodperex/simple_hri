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
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from huggingface_hub import InferenceClient

from simple_hri import prompts

try:
    from simple_hri_interfaces.srv import YesNo
except ImportError:
    print("⚠️ WARNING: simple_hri_interfaces not found. Service will fail if called.")
    class YesNo:
        pass


# Word lists of each language: (doubt, negative, affirmative). Doubt is checked first because
# it contains "no" ("no sé"), and a negation wins over an affirmative word ("claro que no").
KEYWORDS = {
    'es': (
        ['no sé', 'no se', 'no lo sé', 'no lo se', 'no estoy seguro', 'no estoy segura',
         'depende', 'no lo tengo claro'],
        ['no', 'nunca', 'jamás', 'jamas', 'negativo', 'tampoco', 'qué va', 'que va',
         'ni de broma', 'ni hablar'],
        ['sí', 'si', 'vale', 'ok', 'okay', 'claro', 'correcto', 'afirmativo', 'efectivamente',
         'por supuesto', 'desde luego', 'de acuerdo', 'venga', 'adelante', 'me parece bien',
         'perfecto', 'genial'],
    ),
    'en': (
        ["i don't know", 'i dont know', 'not sure', "i'm not sure", 'it depends', 'depends'],
        ['no', 'nope', 'nah', 'never', 'not', 'no way', "don't", 'negative', 'better not'],
        ['yes', 'yeah', 'yep', 'ok', 'okay', 'sure', 'of course', 'alright', 'all right',
         'absolutely', 'agreed', 'fine', 'go ahead', 'sounds good', 'correct', 'right',
         'definitely', 'great', 'perfect'],
    ),
}


class YesNoServiceLocal(Node):
    def __init__(self):
        super().__init__("yesno_service_node")

        self.declare_parameter("prompt_file", prompts.PROMPT_FILE)
        self.declare_parameter("model_id", "Qwen/Qwen2.5-Coder-32B-Instruct")
        self.declare_parameter("use_llm", False)  # By default, use simple pattern matching
        self.use_llm = self.get_parameter("use_llm").get_parameter_value().bool_value
        self.declare_parameter("language", "es")  # Word lists: es or en
        self.language = self.get_parameter("language").get_parameter_value().string_value
        if self.language not in KEYWORDS:
            self.get_logger().warn(f"⚠️ No word lists for '{self.language}': using 'es'")
            self.language = "es"

        self.prompt = prompts.load_prompt(
            self.get_parameter("prompt_file").get_parameter_value().string_value, 'yesno')
        self.system_prompt = self.prompt.system
        self.get_logger().info(f"📄 Prompt loaded from {self.prompt.path}")

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

    def extract_yesno_from_text(self, text: str) -> str:
        """Use Hugging Face to detect yes/no from text, with fallback to pattern matching."""
        
        # Use simple pattern matching if LLM is disabled or not available
        if not self.use_llm or not self.client:
            return self._simple_yesno_detection(text)

        # Try LLM approach first
        prompt_user = self.prompt.user(text=text)

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
            
            # The previous substring check took "UNKNOWN" (it contains "NO") as NO
            return prompts.parse_yesno(completion.choices[0].message.content)
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ LLM inference failed: {e}. Using fallback detection.")
            return self._simple_yesno_detection(text)

    def _simple_yesno_detection(self, text: str) -> str:
        """Fallback method using simple pattern matching on whole words."""
        text_lower = ' ' + ' '.join(re.findall(r"[a-záéíóúüñ']+", text.lower())) + ' '

        def has(patterns):
            return any(f' {p} ' in text_lower for p in patterns)

        doubt, no_patterns, yes_patterns = KEYWORDS[self.language]
        if has(doubt):
            return "UNKNOWN"
        if has(no_patterns):
            return "NO"
        if has(yes_patterns):
            return "YES"
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
