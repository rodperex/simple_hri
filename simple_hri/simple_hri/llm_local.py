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

"""Extract and Yes/No services with a local LLM, run with transformers.

The model is loaded once and serves both services, so that it fits in a small GPU.
"""

import os

# Download the models to ./models (set before importing transformers)
cache_dir = os.path.abspath(os.path.join(os.getcwd(), 'models'))
os.makedirs(cache_dir, exist_ok=True)
os.environ['HF_HOME'] = cache_dir

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402
import torch  # noqa: E402
from transformers import pipeline  # noqa: E402

from simple_hri_interfaces.srv import Extract, YesNo  # noqa: E402
from simple_hri import prompts  # noqa: E402


class LLMLocal(Node):

    def __init__(self, extract=True, yesno=True):
        super().__init__('llm_local_node')

        # Instruction (chat) model of Hugging Face. Options:
        # "Qwen/Qwen2.5-1.5B-Instruct"   # DEFAULT. ~3 GB of GPU memory, 0.3 s per request;
        #                                # 10-30 s on the CPU
        # "Qwen/Qwen2.5-0.5B-Instruct"   # Faster on the CPU, but it makes many mistakes
        # "Qwen/Qwen2.5-3B-Instruct"     # Better, ~6 GB of GPU memory
        self.declare_parameter('model_id', 'Qwen/Qwen2.5-1.5B-Instruct')
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('prompt_file', prompts.PROMPT_FILE)
        # Services offered by this node
        self.declare_parameter('extract', extract)
        self.declare_parameter('yesno', yesno)

        model_id = self.get_parameter('model_id').value
        prompt_file = self.get_parameter('prompt_file').value
        self.extract_prompt = prompts.load_prompt(prompt_file, 'extract')
        self.yesno_prompt = prompts.load_prompt(prompt_file, 'yesno')
        self.get_logger().info(f'📄 Prompts loaded from {self.extract_prompt.path}')

        device = -1  # CPU
        if self.get_parameter('use_gpu').value:
            if torch.cuda.is_available():
                device = 0
                self.get_logger().info('🚀 CUDA detected. Using the GPU.')
            else:
                self.get_logger().warn('⚠️ No CUDA GPU: running on the CPU (slow).')

        self.get_logger().info(f'⏳ Loading local model {model_id}...')
        # Half precision on the GPU: half the memory (a 1.5B model takes ~3 GB)
        dtype = torch.float16 if device == 0 else torch.float32
        self.generator = pipeline('text-generation', model=model_id, device=device,
                                  torch_dtype=dtype)
        self.get_logger().info(f'✅ Model {model_id} loaded.')

        if self.get_parameter('extract').value:
            self.extract_srv = self.create_service(
                Extract, 'extract_service', self.extract_callback)
            self.extract_pub = self.create_publisher(String, '/extracted_text', 10)
            self.get_logger().info('✅ Service "extract_service" ready.')
        if self.get_parameter('yesno').value:
            self.yesno_srv = self.create_service(YesNo, 'yesno_service', self.yesno_callback)
            self.yesno_pub = self.create_publisher(String, '/yesno_result', 10)
            self.get_logger().info('✅ Service "yesno_service" ready.')

    def chat(self, prompt, user, max_new_tokens):
        messages = [
            {'role': 'system', 'content': prompt.system},
            {'role': 'user', 'content': user},
        ]
        result = self.generator(messages, max_new_tokens=max_new_tokens, do_sample=False)
        # The pipeline returns the conversation with the answer of the model at the end
        return result[0]['generated_text'][-1]['content']

    def extract_callback(self, request, response):
        if not request.interest:
            response.result = 'ERROR: Missing interest'
            return response
        if not request.text:
            response.result = 'NONE'
            return response

        self.get_logger().info(f'🎯 Looking for "{request.interest}" in "{request.text}"')
        try:
            answer = self.chat(self.extract_prompt, self.extract_prompt.user(
                interest=request.interest, text=request.text), 40)
            result = prompts.clean_extracted(answer)
        except Exception as e:
            self.get_logger().error(f'❌ Local inference failed: {e}')
            result = f'ERROR: {e}'

        self.get_logger().info(f'✅ Result: {result}')
        self.extract_pub.publish(String(data=result))
        response.result = result
        return response

    def yesno_callback(self, request, response):
        if not request.text:
            response.result = 'ERROR: Empty text'
            return response

        self.get_logger().info(f'🎯 Detecting yes/no in "{request.text}"')
        try:
            answer = self.chat(self.yesno_prompt, self.yesno_prompt.user(text=request.text), 5)
            result = prompts.parse_yesno(answer)
        except Exception as e:
            self.get_logger().error(f'❌ Local inference failed: {e}')
            result = f'ERROR: {e}'

        self.get_logger().info(f'✅ Result: {result}')
        self.yesno_pub.publish(String(data=result))
        response.result = result
        return response


def run(extract, yesno, args=None):
    rclpy.init(args=args)
    try:
        node = LLMLocal(extract, yesno)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    """Extract and Yes/No with one model."""
    run(True, True, args)


def main_extract(args=None):
    """Only Extract."""
    run(True, False, args)


def main_yesno(args=None):
    """Only Yes/No."""
    run(False, True, args)


if __name__ == '__main__':
    main()
