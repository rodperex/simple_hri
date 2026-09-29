import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import os
import tempfile
import uuid

from simple_hri.audio_player import AudioPlayer

class AudioReceiver(Node):
    def __init__(self):
        super().__init__('audio_receiver')

        self.declare_parameter('audio_player', '') # Command used to play (e.g. 'aplay -q', 'pw-play'). '' = auto
        audio_player = self.get_parameter('audio_player').get_parameter_value().string_value
        self.player = AudioPlayer(self.get_logger(), audio_player)

        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'audio_file_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Waiting for WAV file...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received audio data: {len(msg.data)} bytes')
        
        # Unique name: the previous file may still be playing
        output_path = os.path.join(tempfile.gettempdir(), f'received_audio_{uuid.uuid4().hex}.wav')
        
        # Convert list of ints back to bytes and write to file
        with open(output_path, 'wb') as f:
            f.write(bytes(msg.data))
            
        self.get_logger().info(f'File saved to {output_path}')

        self.get_logger().info('Playing received audio...')
        self.player.play(output_path)



def main(args=None):
    rclpy.init(args=args)
    node = AudioReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()