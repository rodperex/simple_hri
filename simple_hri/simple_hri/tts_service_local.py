#!/usr/bin/env python3

import os
import uuid
import torch
import numpy as np
import scipy.io.wavfile
import rclpy
from rclpy.node import Node
from transformers import pipeline

# Import your custom service interface
from simple_hri_interfaces.srv import Speech
from sound_play.libsoundplay import SoundClient
from audio_send_interfaces.srv import SendAudio

class HFTTSService(Node):
    def __init__(self):
        super().__init__("tts_srv_node")

        # Parameters
        self.declare_parameter('lang_code', 'spa') 
        self.declare_parameter('volume', 1.0)
        self.declare_parameter('use_gpu', False) # New param to toggle GPU
        self.declare_parameter('play_sound', True) # If True, use SoundClient to play audio. If False, publish audio data.
        
        lang_code = self.get_parameter('lang_code').get_parameter_value().string_value
        self.volume = self.get_parameter('volume').get_parameter_value().double_value
        use_gpu = self.get_parameter('use_gpu').get_parameter_value().bool_value
        self.play_sound = self.get_parameter('play_sound').get_parameter_value().bool_value

        if not self.play_sound:
            self.get_logger().info("TTS Service configured to PUBLISH audio data instead of playing it.")
            self.audio_send_client = self.create_client(SendAudio, '/trigger_audio_send')
        else:
            self.get_logger().info("TTS Service configured to PLAY audio via SoundClient.")

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
            # Pass device to pipeline
            self.synthesizer = pipeline("text-to-speech", model=model_id, device=self.device)
            self.get_logger().info("Hugging Face Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load {model_id}: {e}. Fallback to English.")
            self.synthesizer = pipeline("text-to-speech", model="facebook/mms-tts-eng", device=self.device)

        # Initialize Sound Client
        self.sound_handle_b = SoundClient(self, blocking=False)

        # Create Service
        self.srv = self.create_service(Speech, "tts_service", self.tts_callback)
        
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
            # 1. Inference
            result = self.synthesizer(reqText)
            audio_data = result['audio']
            sampling_rate = result['sampling_rate']

            # 2. Data Normalization (Ensure float32 is within -1.0 to 1.0)
            # HF output is usually correct, but Transpose if necessary
            if audio_data.ndim > 1:
                audio_data = audio_data.T
            
            # 3. Create Unique Filename to avoid race conditions
            unique_filename = f"tts_{uuid.uuid4().hex}.wav"
            output_path = os.path.join("/tmp", unique_filename)

            # 4. Write WAV
            scipy.io.wavfile.write(output_path, rate=sampling_rate, data=audio_data)
            
            # 5. Play via SoundClient
            # Ensure sound_play node can access /tmp
            if self.play_sound:
                self.sound_handle_b.playWave(output_path, self.volume)

            else:
                if self.audio_send_client.service_is_ready():
                    send_req = SendAudio.Request()
                    send_req.file_path = output_path
                    
                    self.audio_send_client.call_async(send_req)
                    
                else:
                    self.get_logger().warn("Audio send service not available.")

            sResponse.success = True
            sResponse.debug = f"Generated {output_path} via HF MMS-TTS"
            
            # Optional: Clean up old files (garbage collection logic could go here)

        except Exception as e:
            self.get_logger().error(f"TTS Inference/Playback failed: {e}")
            sResponse.success = False
            sResponse.debug = str(e)

        return sResponse
    
def main():
    rclpy.init()
    try:
        tts_service = HFTTSService()
        rclpy.spin(tts_service)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()