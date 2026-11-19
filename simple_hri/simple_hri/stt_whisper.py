#!/usr/bin/env python3

# Copyright 2025 Rodrigo Pérez-Rodríguez
# Licensed under the Apache License, Version 2.0

import rclpy
import numpy as np
import sounddevice as sd
import time
import torch
import whisper
import threading
from rclpy.node import Node
from std_msgs.msg import String

from simple_hri.stt_filter import is_hallucination, keep_segment
from simple_hri.vad_phrase import (
    ENDED, FRAME_SAMPLES, PhraseDetector, SAMPLE_RATE, STARTED, TOO_LONG, WAITING)
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
        self.declare_parameter('max_phrase_secs', 15.0)  # s, longest phrase recorded
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
        self.last_record_timed_out = False
        self.listen_action = ListenActionServer(self, self.listen, self.mic_lock)
        
        self.get_logger().info('✅ STTService inicializado y listo.')

    def record_audio_with_vad(self, max_wait=MAX_WAIT_SECONDS, should_stop=None, on_status=None):
        """
        Record one phrase (see vad_phrase): wait for voice (at most max_wait s), then record
        until grace_period s of silence or max_phrase_secs. Returns int16 audio, empty on
        timeout or cancellation. should_stop() is checked on every frame and on_status()
        reports the stage ("listening", "speech_detected"); both are optional.
        """
        p = self.get_parameter
        detector = PhraseDetector(
            vad_mode=p('vad_mode').value, noise_margin=p('noise_margin').value,
            grace_period=p('grace_period').value, max_phrase_secs=p('max_phrase_secs').value,
            pre_record=p('pre_record').value)
        start_wait_time = time.time()
        self.last_record_timed_out = False

        self.get_logger().info('🎙 Listening... (speak now)')
        if on_status:
            on_status('listening')

        try:
            with sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype='int16') as stream:
                while True:
                    if should_stop and should_stop():
                        self.get_logger().info('⏹ Recording canceled.')
                        return np.array([])

                    frame, overflow = stream.read(FRAME_SAMPLES)
                    if overflow:
                        self.get_logger().warning('⚠️ Audio overflow')
                    event = detector.add(frame)

                    if event == STARTED:
                        self.get_logger().info(
                            f'🔊 Voice detected (background {detector.background_db:.0f} dBFS), '
                            'recording...')
                        if on_status:
                            on_status('speech_detected')
                    elif event == WAITING and time.time() - start_wait_time > max_wait:
                        self.get_logger().info(
                            f'⏰ Timeout: nobody spoke (background {detector.background_db:.0f} '
                            'dBFS).')
                        self.last_record_timed_out = True
                        return np.array([])
                    elif event == ENDED:
                        self.get_logger().info('🛑 End of the phrase.')
                        break
                    elif event == TOO_LONG:
                        self.get_logger().info('🛑 Longest phrase reached.')
                        break

        except Exception as e:
            self.get_logger().error(f'❌ Microphone error: {e}')
            return np.array([])

        return detector.phrase()

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
        """Trabajo de la acción /stt_action (el cerrojo lo toma ListenActionServer)."""
        deadline = time.time() + (max_wait if max_wait > 0 else MAX_WAIT_SECONDS)
        while True:
            audio_data = self.record_audio_with_vad(
                deadline - time.time(), should_stop, on_status)
            if should_stop():
                return False, False, '', 'canceled'
            if len(audio_data) == 0:
                return False, self.last_record_timed_out, '', "No se detectó audio o timeout."

            on_status('transcribing')
            try:
                text = self.transcribe(audio_data)
            except Exception as e:
                self.get_logger().error(f'❌ Error inferencia: {e}')
                return False, False, '', str(e)
            if text:
                return True, False, text, text
            # Lo que oyó Whisper no era voz (ver stt_filter): seguir escuchando el resto de
            # max_wait, para que el ruido no termine la espera a la persona
            if time.time() >= deadline:
                return False, True, '', 'Nadie habló: lo que se oyó no era voz'

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