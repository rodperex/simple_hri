#!/usr/bin/env python3

# Copyright 2024 Antonio Bono
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

from google.cloud import texttospeech
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from hni_interfaces.srv import TextToSpeech

# string text
# ---
# bool success
# string debug

from sound_play.libsoundplay import SoundClient
from audio_send_interfaces.srv import SendAudio

#from sound_play.msg import SoundRequest

from std_msgs.msg import String


class TTSService(Node):
    def __init__(self):
        super().__init__("tts_srv_node")

        self.declare_parameter('play_sound', True) # If True, use SoundClient to play audio. If False, publish audio data.

        self.play_sound = self.get_parameter('play_sound').get_parameter_value().bool_value

        if not self.play_sound:
            self.get_logger().info("TTS Service configured to PUBLISH audio data instead of playing it.")
        
        self.audio_send_client = self.create_client(SendAudio, '/trigger_audio_send')

        # Instantiates a google tts client
        self.client = texttospeech.TextToSpeechClient()

        self.voice = texttospeech.VoiceSelectionParams(
            #language_code="IT-IT", name="it-IT-Neural2-C"  # A female, C male
            #language_code="en-US", name="en-US-Neural2-D"  # C female
            #language_code="en-US", name="en-US-Studio-Q"  # male, O female
            # language_code="en-US", name="en-US-Journey-D"
            language_code="es-ES", name="es-ES-Journey-D"

        )

        # Select the type of audio file you want returned
        self.audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.OGG_OPUS
        )

        self.sound_handle_b = SoundClient(self, blocking=False)

        self.srv = self.create_service(TextToSpeech, "tts_service", self.tts_callback)

        self.volume = 0.9 # from 0.1 to 1.0

        self.get_logger().info("TTSService Server initialized.")

    def tts_callback(self, sRequest, sResponse):
        # response.sum = request.a + request.b
        self.get_logger().info("TTSService Incoming request waiting")

        #self.get_clock().sleep_for(Duration(seconds=30))

        reqText = sRequest.text.strip()
        if reqText:  # not empty string
            # Set the text input to be synthesized
            synthesis_input = texttospeech.SynthesisInput(text=reqText)

            # Perform the text-to-speech request on the text input with the selected
            # voice parameters and audio file type
            response = self.client.synthesize_speech(
                input=synthesis_input, voice=self.voice, audio_config=self.audio_config
            )

            # The response's audio_content is binary.
            with open("/tmp/output.ogg", "wb") as out:
                # Write the response to the output file.
                out.write(response.audio_content)
                self.get_logger().debug('Audio content written to file "output.ogg"')

            self.get_logger().info(f'Playing output.ogg at {self.volume*100}% volume.')
            
            if self.play_sound:
                self.sound_handle_b.playWave("/tmp/output.ogg", self.volume)

            else:
                if self.audio_send_client.service_is_ready():
                    send_req = SendAudio.Request()
                    send_req.file_path = "/tmp/output.ogg"
                    
                    self.audio_send_client.call_async(send_req)
                    
                else:
                    self.get_logger().warn("Audio send service not available.")

            sResponse.success = True

        else:
            sResponse.success = False
            sResponse.debug = "empty text to convert"
            self.get_logger().warn('empty text to convert')

        return sResponse


def main():
    rclpy.init()

    tts_service = TTSService()

    rclpy.spin(tts_service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()