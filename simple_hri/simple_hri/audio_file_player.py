import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from sound_play.libsoundplay import SoundClient
import os

class AudioReceiver(Node):
    def __init__(self):
        super().__init__('audio_receiver')

        self.sound_handle_b = SoundClient(self, blocking=False) 

        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'audio_file_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Waiting for OGG file...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received audio data: {len(msg.data)} bytes')
        
        output_path = os.path.join(os.getcwd(), 'tmp/received_audio.ogg')
        
        # Convert list of ints back to bytes and write to file
        with open(output_path, 'wb') as f:
            f.write(bytes(msg.data))
            
        self.get_logger().info(f'File saved to {output_path}')

        self.get_logger().info('Playing received audio...')
        self.sound_handle_b.playWave(output_path, volume=1.0)



def main(args=None):
    rclpy.init(args=args)
    node = AudioReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()