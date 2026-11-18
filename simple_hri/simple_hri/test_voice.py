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

"""Listen once with /stt_action and repeat what was heard with /tts_action."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from simple_hri_interfaces.action import Listen, Say


class TestVoice(Node):

    def __init__(self):
        super().__init__('test_voice')
        self.listen_client = ActionClient(self, Listen, '/stt_action')
        self.say_client = ActionClient(self, Say, '/tts_action')
        for name, client in (('/stt_action', self.listen_client),
                             ('/tts_action', self.say_client)):
            while not client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info(f'{name} not available, waiting...')
        self.get_logger().info('STT and TTS actions ready.')

    def run_goal(self, client, goal):
        """Send a goal and wait for its result (None if it was rejected)."""
        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        handle = goal_future.result()
        if not handle.accepted:
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result

    def run(self):
        self.get_logger().info('Listening (STT)... speak now')
        heard = self.run_goal(self.listen_client, Listen.Goal(max_wait=10.0))
        if heard is None or not heard.success:
            message = heard.message if heard is not None else 'goal rejected'
            self.get_logger().error(f'STT error: {message}')
            return
        self.get_logger().info(f'Transcription: {heard.text}')

        self.get_logger().info('Repeating it (TTS)...')
        said = self.run_goal(self.say_client, Say.Goal(text=heard.text))
        if said is not None and said.success:
            self.get_logger().info('TTS finished')
        else:
            message = said.message if said is not None else 'goal rejected'
            self.get_logger().error(f'TTS error: {message}')


def main(args=None):
    rclpy.init(args=args)
    node = TestVoice()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
