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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from simple_hri_interfaces.srv import YesNo
from simple_hri.ollama_llm import DEFAULT_MODEL, OllamaLLM
from simple_hri import prompts


class YesNoServiceOllama(Node):
    """Yes/no service with an LLM served by Ollama (local GPU or remote server)."""

    def __init__(self):
        super().__init__('yesno_service_node')

        self.declare_parameter('prompt_file', prompts.PROMPT_FILE)
        self.declare_parameter('host', '')  # '' = OLLAMA_HOST or http://localhost:11434
        self.declare_parameter('model', DEFAULT_MODEL)
        self.declare_parameter('timeout', 60.0)
        self.declare_parameter('keep_alive', '')  # '' = server default (5 minutes)
        self.declare_parameter('preload', True)

        self.prompt = prompts.load_prompt(
            self.get_parameter('prompt_file').value, 'yesno')
        self.system_prompt = self.prompt.system
        self.get_logger().info(f'Prompt loaded from {self.prompt.path}')

        self.llm = OllamaLLM(
            self.get_logger(),
            self.get_parameter('host').value,
            self.get_parameter('model').value,
            self.get_parameter('timeout').value,
            self.get_parameter('keep_alive').value)
        if self.llm.check() and self.get_parameter('preload').value:
            self.llm.preload()

        self.srv = self.create_service(YesNo, 'yesno_service', self.yesno_callback)
        self.pub = self.create_publisher(String, '/yesno_result', 10)

        self.get_logger().info('✅ Service "yesno_service" (Ollama) ready.')

    def yesno_callback(self, request, response):
        if not request.text:
            response.result = 'ERROR: Empty text'
            return response

        self.get_logger().info(f'🎯 Detecting yes/no in "{request.text}"')

        try:
            answer = self.llm.chat(
                self.system_prompt, self.prompt.user(text=request.text), max_tokens=10)
        except Exception as e:
            self.get_logger().error(f'❌ Ollama request failed: {e}')
            response.result = f'ERROR: {e}'
            return response

        result = prompts.parse_yesno(answer)

        self.get_logger().info(f'✅ Result: {result} (model answered "{answer}")')
        self.pub.publish(String(data=result))
        response.result = result
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YesNoServiceOllama()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
