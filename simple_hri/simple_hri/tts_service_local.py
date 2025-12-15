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
import scipy.io.wavfile
from transformers import pipeline

# Import your custom service interface
from hni_interfaces.srv import TextToSpeech
from sound_play.libsoundplay import SoundClient

class HFTTSService(Node):
    def __init__(self):
        super().__init__("tts_srv_node")

        # 'spa' = Spanish, 'eng' = English, 'ita' = Italian, 'fra' = French, etc.
        self.declare_parameter('lang_code', 'spa') 
        self.declare_parameter('volume', 1.0)
        
        lang_code = self.get_parameter('lang_code').get_parameter_value().string_value
        self.volume = self.get_parameter('volume').get_parameter_value().double_value

        self.get_logger().info(f"Loading Hugging Face Model for language: {lang_code}...")

        model_id = f"facebook/mms-tts-{lang_code}"
        
        try:
            self.synthesizer = pipeline("text-to-speech", model=model_id)
            self.get_logger().info("Hugging Face Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model {model_id}. Error: {e}")
            # Fallback to English if specific language fails
            self.synthesizer = pipeline("text-to-speech", model="facebook/mms-tts-eng")

        # 3. Initialize Sound Client
        self.sound_handle_b = SoundClient(self, blocking=False)

        # 4. Create Service
        self.srv = self.create_service(TextToSpeech, "tts_service", self.tts_callback)
        
        self.get_logger().info("TTSService (Hugging Face) initialized.")

    def tts_callback(self, sRequest, sResponse):
        self.get_logger().info("TTSService Incoming request waiting")

        reqText = sRequest.text.strip()
        
        if reqText:
            try:
                result = self.synthesizer(reqText)
                
                audio_data = result['audio']
                sampling_rate = result['sampling_rate']

                output_file = "/tmp/output.wav"

                scipy.io.wavfile.write(output_file, rate=sampling_rate, data=audio_data.T)
                
                self.get_logger().debug(f'Audio content written to {output_file}')

                self.get_logger().info(f'Playing {output_file} at {self.volume*100}% volume.')
                self.sound_handle_b.playWave(output_file, self.volume)

                sResponse.success = True
                sResponse.debug = "Generated via HF MMS-TTS"

            except Exception as e:
                self.get_logger().error(f"TTS Inference/Playback failed: {e}")
                sResponse.success = False
                sResponse.debug = str(e)
        else:
            sResponse.success = False
            sResponse.debug = "empty text to convert"
            self.get_logger().warn('empty text to convert')

        return sResponse


def main():
    rclpy.init()
    tts_service = HFTTSService()
    rclpy.spin(tts_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()