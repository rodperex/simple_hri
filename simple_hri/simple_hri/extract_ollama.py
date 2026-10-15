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

from simple_hri_interfaces.srv import Extract
from simple_hri.ollama_llm import DEFAULT_MODEL, OllamaLLM
from simple_hri import prompts


class ExtractServiceOllama(Node):
    """Extract service with an LLM served by Ollama (local GPU or remote server)."""

    def __init__(self):
        super().__init__('extract_service_node')

        self.declare_parameter('prompt_file', prompts.PROMPT_FILE)
        self.declare_parameter('host', '')  # '' = OLLAMA_HOST or http://localhost:11434
        self.declare_parameter('model', DEFAULT_MODEL)
        self.declare_parameter('timeout', 60.0)
        self.declare_parameter('keep_alive', '')  # '' = server default (5 minutes)
        self.declare_parameter('preload', True)

        self.prompt = prompts.load_prompt(
            self.get_parameter('prompt_file').value, 'extract')
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

        self.srv = self.create_service(Extract, 'extract_service', self.extract_callback)
        self.pub = self.create_publisher(String, '/extracted_text', 10)

        self.get_logger().info('✅ Service "extract_service" (Ollama) ready.')

    def extract_callback(self, request, response):
        if not request.interest:
            response.result = 'ERROR: Missing interest'
            return response
        if not request.text:
            response.result = 'NONE'
            return response

        self.get_logger().info(f'🎯 Looking for "{request.interest}" in "{request.text}"')

        prompt_user = self.prompt.user(interest=request.interest, text=request.text)
        try:
            extracted = prompts.clean_extracted(self.llm.chat(self.system_prompt, prompt_user))
        except Exception as e:
            self.get_logger().error(f'❌ Ollama request failed: {e}')
            response.result = f'ERROR: {e}'
            return response

        self.get_logger().info(f'✅ Result: {extracted}')
        self.pub.publish(String(data=extracted))
        response.result = extracted
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ExtractServiceOllama()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
