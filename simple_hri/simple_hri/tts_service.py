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
import math
import threading
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from simple_hri_interfaces.srv import Speech

# string text
# ---
# bool success
# string debug

from audio_send_interfaces.srv import SendAudio

from std_msgs.msg import String

from simple_hri.audio_player import AudioPlayer
from simple_hri.voice_actions import (
    SayActionServer, audio_file_duration, estimate_duration, spin_multithreaded)


class TTSService(Node):
    def __init__(self):
        super().__init__("tts_srv_node")

        self.declare_parameter('play_sound', True) # If True, play the audio here. If False, publish audio data.
        self.declare_parameter('audio_player', '') # Command used to play (e.g. 'aplay -q', 'pw-play'). '' = auto

        self.play_sound = self.get_parameter('play_sound').get_parameter_value().bool_value
        audio_player = self.get_parameter('audio_player').get_parameter_value().string_value

        if not self.play_sound:
            self.get_logger().info("TTS Service configured to PUBLISH audio data instead of playing it.")
        
        self.audio_send_client = self.create_client(SendAudio, '/trigger_audio_send')

        # Instantiates a google tts client
        self.client = texttospeech.TextToSpeechClient()

        self.voice = texttospeech.VoiceSelectionParams(
            #language_code="IT-IT", name="it-IT-Neural2-C"  # A female, C male
            language_code="en-US", name="en-US-Neural2-D"  # C female
            #language_code="en-US", name="en-US-Studio-Q"  # male, O female
            # language_code="en-US", name="en-US-Journey-D"
            # language_code="es-ES", name="es-ES-Journey-D"

        )

        self.volume = 0.9 # from 0.1 to 1.0

        # WAV (LINEAR16) so that any player (aplay included) can play it.
        # The volume is applied by Google when synthesizing.
        self.audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.LINEAR16,
            volume_gain_db=20 * math.log10(max(self.volume, 0.1))
        )

        self.player = AudioPlayer(self.get_logger(), audio_player)

        self.srv = self.create_service(Speech, "tts_service", self.tts_callback)

        # Action /tts_action: same work as the service, but it finishes when playback ends
        # and can be canceled. The lock serializes synthesis between service and action.
        self.synth_lock = threading.Lock()
        self.say_action = SayActionServer(self, self.speak, self.stop_speaking, self.synth_lock)

        self.get_logger().info("✅ TTSService Server initialized.")

    def tts_callback(self, sRequest, sResponse):
        # response.sum = request.a + request.b
        self.get_logger().info("TTSService Incoming request waiting")

        #self.get_clock().sleep_for(Duration(seconds=30))

        reqText = sRequest.text.strip()
        if reqText:  # not empty string
            with self.synth_lock:
                self.synthesize_and_play(reqText)

            sResponse.success = True

        else:
            sResponse.success = False
            sResponse.debug = "empty text to convert"
            self.get_logger().warn('empty text to convert')

        return sResponse

    def synthesize_and_play(self, text):
        """Synthesize the text, start playing it (non-blocking) and return the file path."""
        # Set the text input to be synthesized
        synthesis_input = texttospeech.SynthesisInput(text=text)

        # Perform the text-to-speech request on the text input with the selected
        # voice parameters and audio file type
        response = self.client.synthesize_speech(
            input=synthesis_input, voice=self.voice, audio_config=self.audio_config
        )

        output_path = f"/tmp/tts_{uuid.uuid4().hex}.wav"

        # The response's audio_content is binary.
        with open(output_path, "wb") as out:
            # Write the response to the output file.
            out.write(response.audio_content)
            self.get_logger().debug(f'Audio content written to file "{output_path}"')

        self.get_logger().info(f'Playing {output_path} at {self.volume*100}% volume.')

        if self.play_sound:
            self.player.play(output_path)

        else:
            if self.audio_send_client.service_is_ready():
                send_req = SendAudio.Request()
                send_req.file_path = output_path

                self.audio_send_client.call_async(send_req)

            else:
                self.get_logger().warn("Audio send service not available.")

        return output_path

    def speak(self, text):
        """Work of the /tts_action action (SayActionServer takes the lock)."""
        text = text.strip()
        if not text:
            return False, 0.0, "empty text to convert"
        try:
            output_path = self.synthesize_and_play(text)
        except Exception as e:
            self.get_logger().error(f'TTS failed: {e}')
            return False, 0.0, str(e)
        # Real duration of the WAV file if it can be read; otherwise, estimated from the text
        duration = audio_file_duration(output_path)
        if duration is None:
            duration = estimate_duration(text)
        return True, duration, f"Generated {output_path}"

    def stop_speaking(self):
        """Stop the playback (used when the /tts_action goal is canceled)."""
        if self.play_sound:
            self.player.stop()
        else:
            self.get_logger().warn('Audio sent to another device: playback cannot be stopped.')


def main():
    rclpy.init()

    tts_service = TTSService()

    try:
        spin_multithreaded(tts_service)
    finally:
        tts_service.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()