#!/usr/bin/env python3

# Copyright 2026 Rodrigo Pérez-Rodríguez
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

from huggingface_hub import InferenceClient
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from simple_hri_interfaces.srv import YesNo
from simple_hri import prompts


class YesNoHF(Node):
    """Yes/no service with an LLM of the Hugging Face Inference Providers (needs credits)."""

    def __init__(self):
        super().__init__('yesno_service_node')

        self.declare_parameter('prompt_file', prompts.PROMPT_FILE)
        self.declare_parameter('model_id', 'meta-llama/Meta-Llama-3-8B-Instruct')

        self.prompt = prompts.load_prompt(
            self.get_parameter('prompt_file').value, 'yesno')
        self.system_prompt = self.prompt.system
        self.get_logger().info(f'Prompt loaded from {self.prompt.path}')
        self.model_id = self.get_parameter('model_id').value

        token = os.getenv('HF_TOKEN')
        if not token:
            self.get_logger().fatal("❌ HF_TOKEN not set. Run: export HF_TOKEN='your_token'")
        self.client = InferenceClient(api_key=token) if token else None

        self.srv = self.create_service(YesNo, 'yesno_service', self.yesno_callback)
        self.pub = self.create_publisher(String, '/yesno_result', 10)

        self.get_logger().info(f'✅ Service "yesno_service" (Hugging Face, {self.model_id}) ready.')

    def yesno_callback(self, request, response):
        if not request.text:
            response.result = 'ERROR: Empty text'
            return response
        if not self.client:
            response.result = 'ERROR: HF_TOKEN not set'
            return response

        self.get_logger().info(f'🎯 Detecting yes/no in "{request.text}"')

        try:
            completion = self.client.chat_completion(
                model=self.model_id,
                messages=[
                    {'role': 'system', 'content': self.system_prompt},
                    {'role': 'user', 'content': self.prompt.user(text=request.text)},
                ],
                max_tokens=10,
                temperature=0.0,
            )
        except Exception as e:
            self.get_logger().error(f'❌ Hugging Face request failed: {e}')
            response.result = f'ERROR: {e}'
            return response

        result = prompts.parse_yesno(completion.choices[0].message.content)
        self.get_logger().info(f'✅ Result: {result}')
        self.pub.publish(String(data=result))
        response.result = result
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YesNoHF()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
