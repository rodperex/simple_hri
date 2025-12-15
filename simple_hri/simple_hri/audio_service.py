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

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import os

# Import your custom service definition
# Assuming your package is named 'my_audio_pkg'
from audio_send_interfaces.srv import SendAudio 

class AudioServiceNode(Node):
    def __init__(self):
        super().__init__('audio_service_node')
        
        self.srv = self.create_service(
            SendAudio, 
            'trigger_audio_send', 
            self.send_audio_callback
        )
        
        self.publisher_ = self.create_publisher(
            UInt8MultiArray, 
            'audio_file_data', 
            10
        )
        
        self.get_logger().info('Service "trigger_audio_send" is ready.')

    def send_audio_callback(self, request, response):
        path = request.file_path
        self.get_logger().info(f'Request received to send: {path}')

        if not os.path.exists(path):
            response.success = False
            response.message = f"File not found: {path}"
            self.get_logger().error(response.message)
            return response

        try:
            with open(path, 'rb') as f:
                file_content = f.read()

            msg = UInt8MultiArray()
            msg.data = list(file_content) # Convert bytes to list[int]

            self.publisher_.publish(msg)
            
            response.success = True
            response.message = f"Successfully published {len(msg.data)} bytes."
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"Error reading/publishing file: {str(e)}"
            self.get_logger().error(response.message)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = AudioServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()