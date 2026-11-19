#!/usr/bin/env python3

# Copyright 2025 Rodrigo Pérez-Rodríguez
# Licensed under the Apache License, Version 2.0

import rclpy
import numpy as np
import sounddevice as sd
import torch
import whisper
import threading
from rclpy.node import Node
from std_msgs.msg import String

from simple_hri.stt_filter import is_hallucination, keep_segment
from simple_hri.vad_phrase import PhraseDetector, SAMPLE_RATE, listen, webrtc_speech
from simple_hri.voice_actions import ListenActionServer, spin_multithreaded

MAX_WAIT_SECONDS = 10.0  # longest wait for somebody to start speaking

MODEL_NAME = "small" # "tiny", "base", "small", "medium", "large"; tiny garbles Spanish phrases

class STTService(Node):
    def __init__(self):
        super().__init__("stt_service_node")
        
        import os
        # Bigger models transcribe better, but are slower without a GPU
        self.declare_parameter('model', MODEL_NAME)
        self.declare_parameter('language', 'es')  # Language spoken: es, en...
        # cpu, cuda or auto. cpu by default: it leaves the GPU to the TTS and the LLM
        self.declare_parameter('device', 'cpu')
        # Discard what Whisper invents from noise (see stt_filter); false only to debug
        self.declare_parameter('hallucination_filter', True)
        self.hallucination_filter = self.get_parameter('hallucination_filter').value
        # Phrase detection (see vad_phrase)
        self.declare_parameter('vad_mode', 2)  # webrtcvad: 0 (takes noise for voice) to 3
        self.declare_parameter('noise_margin', 6.0)  # dB above the background that is voice
        self.declare_parameter('grace_period', 1.0)  # s of silence that end the phrase
        self.declare_parameter('max_phrase_secs', 60.0)  # s, longest phrase; 0 = no limit
        self.declare_parameter('pre_record', 0.5)  # s of audio kept before the voice
        self.language = self.get_parameter('language').get_parameter_value().string_value
        model_name = self.get_parameter('model').get_parameter_value().string_value
        self.get_logger().info(f'⏳ Cargando modelo Whisper ({model_name})...')
        # Buscar directorio models/ en la jerarquía o usar variable de entorno
        # Usar siempre ./models relativo al current working directory
        whisper_cache_dir = os.path.abspath(os.path.join(os.getcwd(), 'models'))
        os.makedirs(whisper_cache_dir, exist_ok=True)
        device = self.get_parameter('device').get_parameter_value().string_value
        if device in ('auto', 'cuda') and not torch.cuda.is_available():
            if device == 'cuda':
                self.get_logger().warn('No CUDA GPU: Whisper runs on the CPU.')
            device = 'cpu'
        elif device == 'auto':
            device = 'cuda'
        self.get_logger().info(f'Whisper on {device}')
        self.whisper_model = whisper.load_model(model_name, device=device,
                                                download_root=whisper_cache_dir)
        
        # Publisher
        self.pub = self.create_publisher(String, "/listened_text", 10)

        # Acción /stt_action. El cerrojo evita que dos objetivos usen el micrófono a la vez.
        self.mic_lock = threading.Lock()
        self.listen_action = ListenActionServer(self, self.listen, self.mic_lock)
        
        self.get_logger().info('✅ STTService inicializado y listo.')

    def new_detector(self, background):
        """Return a PhraseDetector with the parameters of the node (see vad_phrase)."""
        p = self.get_parameter
        return PhraseDetector(
            webrtc_speech(p('vad_mode').value), noise_margin=p('noise_margin').value,
            grace_period=p('grace_period').value, max_phrase_secs=p('max_phrase_secs').value,
            pre_record=p('pre_record').value, background=background)

    def transcribe(self, audio_data):
        """Transcribe con Whisper y publica en /listened_text; '' si no es voz (ver stt_filter)."""
        # Preprocesar para Whisper (int16 -> float32 normalizado entre -1 y 1)
        audio_float = audio_data.astype(np.float32) / 32768.0

        self.get_logger().info("🧠 Procesando con Whisper...")

        # FP16 only on the GPU: the CPU does not support it
        result = self.whisper_model.transcribe(
            audio_float,
            language=self.language,
            fp16=self.whisper_model.device.type == 'cuda',
            condition_on_previous_text=False,
        )

        heard = result["text"].strip()
        if self.hallucination_filter:
            text = ''.join(s['text'] for s in result['segments']
                           if keep_segment(s['no_speech_prob'])).strip()
            if is_hallucination(text):
                self.get_logger().info(f'Descartado, no es voz: "{heard}"')
                return ''
        else:
            text = heard
        self.get_logger().info(f'📝 Resultado: "{text}"')

        msg = String()
        msg.data = text
        self.pub.publish(msg)
        return text

    def listen(self, max_wait, should_stop, on_status):
        """Work of the /stt_action action (ListenActionServer takes the lock)."""
        return listen(
            lambda: sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype='int16'),
            self.new_detector, self.transcribe,
            max_wait if max_wait > 0 else MAX_WAIT_SECONDS, should_stop, on_status,
            self.get_logger())


def main():
    rclpy.init()
    node = STTService()
    try:
        spin_multithreaded(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
