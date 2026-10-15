#!/usr/bin/env python3

import os
import threading
import uuid

# Configurar caché de transformers ANTES de importar
cache_dir = os.path.abspath(os.path.join(os.getcwd(), 'models'))
os.makedirs(cache_dir, exist_ok=True)
os.environ['HF_HOME'] = cache_dir

import torch
import numpy as np
import scipy.io.wavfile
import rclpy
from rclpy.node import Node
from transformers import pipeline

# Import your custom service interface
from simple_hri_interfaces.srv import Speech
from audio_send_interfaces.srv import SendAudio

from simple_hri.audio_player import AudioPlayer
from simple_hri.voice_actions import SayActionServer, spin_multithreaded

class HFTTSService(Node):
    def __init__(self):
        super().__init__("tts_srv_node")

        # Parameters
        self.declare_parameter('lang_code', 'spa') 
        self.declare_parameter('volume', 1.0)
        self.declare_parameter('use_gpu', False) # New param to toggle GPU
        self.declare_parameter('play_sound', True) # If True, play the audio here. If False, publish audio data.
        self.declare_parameter('audio_player', '') # Command used to play (e.g. 'aplay -q', 'pw-play'). '' = auto
        
        lang_code = self.get_parameter('lang_code').get_parameter_value().string_value
        self.volume = self.get_parameter('volume').get_parameter_value().double_value
        use_gpu = self.get_parameter('use_gpu').get_parameter_value().bool_value
        self.play_sound = self.get_parameter('play_sound').get_parameter_value().bool_value
        audio_player = self.get_parameter('audio_player').get_parameter_value().string_value

        if not self.play_sound:
            self.get_logger().info("TTS Service configured to PUBLISH audio data instead of playing it.")
            self.audio_send_client = self.create_client(SendAudio, '/trigger_audio_send')
        else:
            self.get_logger().info("TTS Service configured to PLAY audio.")

        # Device selection
        self.device = -1 # CPU
        if use_gpu and torch.cuda.is_available():
            self.device = 0 # First GPU
            self.get_logger().info("CUDA detected. Using GPU for inference.")
        elif use_gpu:
            self.get_logger().warn("GPU requested but CUDA not available. Falling back to CPU.")

        model_id = f"facebook/mms-tts-{lang_code}"
        self.get_logger().info(f"Loading Hugging Face Model: {model_id}...")
        
        try:
            self.synthesizer = pipeline("text-to-speech", model=model_id, device=self.device)
            self.get_logger().info("Hugging Face Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load {model_id}: {e}. Fallback to English.")
            self.synthesizer = pipeline("text-to-speech", model="facebook/mms-tts-eng", device=self.device)

        self.player = AudioPlayer(self.get_logger(), audio_player)

        # Create Service
        self.srv = self.create_service(Speech, "tts_service", self.tts_callback)

        # Action /tts_action: same work as the service, but it finishes when playback ends
        # and can be canceled. The lock serializes synthesis between service and action.
        self.synth_lock = threading.Lock()
        self.say_action = SayActionServer(self, self.speak, self.stop_speaking, self.synth_lock)
        
        self.get_logger().info("TTSService (Hugging Face) initialized.")

    def tts_callback(self, sRequest, sResponse):
        reqText = sRequest.text.strip()
        
        if not reqText:
            sResponse.success = False
            sResponse.debug = "Empty text provided"
            self.get_logger().warn(sResponse.debug)
            return sResponse

        self.get_logger().info(f"Processing TTS: '{reqText[:20]}...'")

        try:
            with self.synth_lock:
                output_path, _ = self.synthesize_and_play(reqText)

            sResponse.success = True
            sResponse.debug = f"Generated {output_path} via HF MMS-TTS"
            
            # Optional: Clean up old files (garbage collection logic could go here)

        except Exception as e:
            self.get_logger().error(f"TTS Inference/Playback failed: {e}")
            sResponse.success = False
            sResponse.debug = str(e)

        return sResponse

    def synthesize_and_play(self, text):
        """Synthesize the text, start playing it (non-blocking) and return (path, duration)."""
        # 1. Inference
        result = self.synthesizer(text)
        audio_data = result['audio']
        sampling_rate = result['sampling_rate']

        # 2. Data Normalization (Ensure float32 is within -1.0 to 1.0)
        # HF output is usually correct, but Transpose if necessary
        if audio_data.ndim > 1:
            audio_data = audio_data.T
        duration = audio_data.shape[0] / sampling_rate

        # 3. Create Unique Filename to avoid race conditions
        unique_filename = f"tts_{uuid.uuid4().hex}.wav"
        output_path = os.path.join("/tmp", unique_filename)

        # 4. Write a 16-bit PCM WAV with the volume applied (players such as aplay do not
        # handle volume, and not every device accepts float samples)
        audio_data = np.clip(audio_data * self.volume, -1.0, 1.0)
        scipy.io.wavfile.write(output_path, rate=sampling_rate,
                               data=(audio_data * 32767).astype(np.int16))

        # 5. Play
        if self.play_sound:
            self.player.play(output_path)

        else:
            if self.audio_send_client.service_is_ready():
                send_req = SendAudio.Request()
                send_req.file_path = output_path

                self.audio_send_client.call_async(send_req)

            else:
                self.get_logger().warn("Audio send service not available.")

        return output_path, duration

    def speak(self, text):
        """Work of the /tts_action action (SayActionServer takes the lock)."""
        text = text.strip()
        if not text:
            return False, 0.0, "Empty text provided"
        self.get_logger().info(f"Processing TTS (action): '{text[:20]}...'")
        try:
            output_path, duration = self.synthesize_and_play(text)
            return True, duration, f"Generated {output_path} via HF MMS-TTS"
        except Exception as e:
            self.get_logger().error(f"TTS Inference/Playback failed: {e}")
            return False, 0.0, str(e)

    def stop_speaking(self):
        """Stop the playback (used when the /tts_action goal is canceled)."""
        if self.play_sound:
            self.player.stop()
        else:
            self.get_logger().warn('Audio sent to another device: playback cannot be stopped.')


def main():
    rclpy.init()
    try:
        tts_service = HFTTSService()
        spin_multithreaded(tts_service)
    except KeyboardInterrupt:  # Ctrl+C while the model is loading
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()