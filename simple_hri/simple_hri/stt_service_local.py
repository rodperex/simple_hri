#!/usr/bin/env python3

# Copyright 2025 Rodrigo Pérez-Rodríguez
# Licensed under the Apache License, Version 2.0

import rclpy
import numpy as np
import sounddevice as sd
import webrtcvad
import time
import whisper
import collections
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import String
from scipy.io.wavfile import write

# --- Configuración ---
SAMPLE_RATE = 16000
FRAME_DURATION = 30  # ms
CHANNELS = 1
VAD_SENSITIVITY = 1  # 0=permisivo, 3=estricto (1 es balanceado para ambientes reales)
SILENCE_DURATION = 1.0  # Segundos de silencio para cortar después de hablar
PRE_RECORD_BUFFER = 0.5 # Segundos de audio a guardar antes de que se detecte voz
MAX_WAIT_SECONDS = 10.0 # Tiempo máximo esperando a que alguien empiece a hablar

MODEL_NAME = "small" # "tiny", "base", "small", "medium", "large"

class STTService(Node):
    def __init__(self):
        super().__init__("stt_service_node")
        
        self.get_logger().info(f'⏳ Cargando modelo Whisper ({MODEL_NAME})...')
        self.whisper_model = whisper.load_model(MODEL_NAME)
        
        # Servicio y Publisher
        self.srv = self.create_service(SetBool, "stt_service", self.stt_callback)
        self.pub = self.create_publisher(String, "/listened_text", 10)
        
        # Inicializar VAD
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(VAD_SENSITIVITY)
        
        self.get_logger().info('✅ STTService inicializado y listo.')

    def record_audio_with_vad(self):
        """
        Graba audio utilizando VAD. Mantiene un 'ring buffer' para no perder
        el inicio de la frase y graba continuamente hasta detectar silencio.
        """
        frame_length = int(SAMPLE_RATE * FRAME_DURATION / 1000) # Samples per frame
        
        # Buffer circular para guardar audio PREVIO a la detección (evita cortar la primera sílaba)
        maxlen_pre = int((SAMPLE_RATE / frame_length) * PRE_RECORD_BUFFER)
        pre_buffer = collections.deque(maxlen=maxlen_pre)
        
        recorded_frames = []
        triggered = False
        start_wait_time = time.time()
        last_voice_time = None

        self.get_logger().info('🎙 Escuchando... (Hable ahora)')

        try:
            with sd.InputStream(samplerate=SAMPLE_RATE, channels=CHANNELS, dtype='int16') as stream:
                while True:
                    frame, overflow = stream.read(frame_length)
                    if overflow:
                        self.get_logger().warning("⚠️ Audio overflow")
                    
                    frame = frame.flatten()
                    frame_bytes = frame.tobytes()
                    
                    # Chequeo de seguridad: VAD requiere frames de 10, 20 o 30ms
                    try:
                        is_speech = self.vad.is_speech(frame_bytes, SAMPLE_RATE)
                    except Exception as e:
                        self.get_logger().warning(f"VAD Error: {e}")
                        is_speech = False

                    current_time = time.time()

                    if not triggered:
                        # --- FASE 1: ESPERANDO VOZ ---
                        pre_buffer.append(frame)
                        
                        if is_speech:
                            self.get_logger().info('🔊 Voz detectada, grabando...')
                            triggered = True
                            last_voice_time = current_time
                            # Volcamos el buffer previo para recuperar el inicio de la frase
                            recorded_frames.extend(pre_buffer)
                        
                        # Timeout si nadie habla
                        elif (current_time - start_wait_time) > MAX_WAIT_SECONDS:
                            self.get_logger().info('⏰ Timeout: Nadie habló.')
                            return np.array([])
                    else:
                        # --- FASE 2: GRABANDO ---
                        recorded_frames.append(frame)
                        
                        if is_speech:
                            last_voice_time = current_time
                        
                        # Si ha pasado mucho tiempo desde la última voz, cortamos
                        if (current_time - last_voice_time) > SILENCE_DURATION:
                            self.get_logger().info('🛑 Fin de frase detectado.')
                            break

        except Exception as e:
            self.get_logger().error(f"❌ Error micrófono: {e}")
            return np.array([])

        if recorded_frames:
            audio_data = np.concatenate(recorded_frames)
            # Opcional: Guardar para debug
            # write("/tmp/debug_audio.wav", SAMPLE_RATE, audio_data)
            return audio_data
        else:
            return np.array([])

    def stt_callback(self, sRequest, sResponse):
        if not sRequest.data:
            sResponse.success = False
            sResponse.message = "Envia 'True' para comenzar a escuchar."
            return sResponse

        # 1. Grabar
        audio_data = self.record_audio_with_vad()
        
        if len(audio_data) == 0:
            sResponse.success = False
            sResponse.message = "No se detectó audio o timeout."
            return sResponse

        try:
            # 2. Preprocesar para Whisper (int16 -> float32 normalizado entre -1 y 1)
            audio_float = audio_data.astype(np.float32) / 32768.0
            
            # 3. Transcribir
            self.get_logger().info("🧠 Procesando con Whisper...")
            
            # 'fp16=False' es necesario si corres en CPU. Si tienes GPU, quítalo o pon True.
            result = self.whisper_model.transcribe(
                audio_float, 
                language="es", 
                fp16=False 
            )

            text = result["text"].strip()
            self.get_logger().info(f'📝 Resultado: "{text}"')

            # 4. Publicar y Responder
            msg = String()
            msg.data = text
            self.pub.publish(msg)

            sResponse.success = True
            sResponse.message = text

        except Exception as e:
            self.get_logger().error(f'❌ Error inferencia: {e}')
            sResponse.success = False
            sResponse.message = str(e)

        return sResponse

def main():
    rclpy.init()
    node = STTService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()