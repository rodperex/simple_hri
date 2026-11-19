# Copyright 2026 Rodrigo Pérez-Rodríguez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Listen for one phrase in a noisy room: used by the three STT nodes.

A voice detector alone (webrtcvad, or Silero in stt_faster_whisper) takes room noise for
voice: with people talking around, a fan or a car it marks most frames as voice, the phrase
never gets 1 s of silence and the recording never ends. These rules make it robust:

  - A frame is voice only if the detector says so AND it is louder than the background
    (the level above 100 Hz, where the voice is, not the rumble of an engine or a fan):
    noise_margin dB to start the phrase, half of it to go on (the voice of a person who
    keeps speaking varies, and must not be cut). The person speaks close to the
    microphone, louder than the room.
  - The background is the median level of the last 3 s while waiting for the person. It
    is not measured while recording: in a long answer with short pauses, the pauses still
    hold the tail of the voice, the background would rise and the person would be cut.
  - The phrase starts when most frames of the last 0.3 s are voice (one frame is not
    enough), and ends when almost none of the last grace_period seconds is voice (a click
    does not keep it open, and a pause to think shorter than grace_period does not end
    it).
  - It never lasts more than max_phrase_secs (60 s by default; 0 = no limit): a safety
    net if noise keeps the phrase open. A person who keeps speaking is never cut before.

PhraseDetector applies these rules frame by frame. listen() is the whole Listen action: it
keeps the microphone open until a phrase is transcribed. When the transcription turns out
not to be speech (see stt_filter), it has gone on listening meanwhile, with the background
already measured, so a person who starts speaking while Whisper works is not lost.
"""

import collections
import threading
import time

import numpy as np
import webrtcvad

SAMPLE_RATE = 16000
FRAME_SAMPLES = 480  # 30 ms; webrtcvad accepts 10, 20 or 30 ms frames

BACKGROUND_SECS = 3.0  # window that measures the background level while waiting
WARMUP_SECS = 0.5  # background measured before a phrase can start
START_SECS = 0.3  # window that starts the phrase...
START_RATIO = 0.7  # ...when this fraction of its frames are voice
# The phrase ends when at most this fraction of the last grace_period is voice. 0.2 ended long
# answers at pauses of 0.7 s; lower than 0.1, people talking around kept it open for minutes
END_RATIO = 0.1
KEEP_MARGIN = 0.5  # fraction of noise_margin that keeps a phrase going once it has started
# The level ignores what is below this frequency. In a car (81% of the noise below 100 Hz) it
# heard three times more phrases; 150 Hz took more chatter of a class for voice
LOW_CUT_HZ = 100

# Events returned by PhraseDetector.add()
WAITING = 'waiting'  # nobody speaks yet
STARTED = 'started'  # the phrase has just started
RECORDING = 'recording'  # inside the phrase
ENDED = 'ended'  # end of the phrase (silence)
TOO_LONG = 'too_long'  # end of the phrase (max_phrase_secs, if it is not 0)


def level_db(frame):
    """
    Return the level of a frame in dBFS (0 = full scale), above LOW_CUT_HZ.

    The frame is int16, or float in [-1, 1]. Below 100 Hz there is hardly any voice, but
    there is the rumble of an engine, the hum of a fan or the vibration of the table, which
    would hide the voice.
    """
    samples = frame.astype(np.float32)
    if frame.dtype == np.int16:
        samples /= 32768.0
    spectrum = np.abs(np.fft.rfft(samples)) ** 2
    freqs = np.fft.rfftfreq(len(samples), 1 / SAMPLE_RATE)
    power = 2 * spectrum[freqs >= LOW_CUT_HZ].sum() / len(samples) ** 2  # Parseval
    return 10 * np.log10(power + 1e-12)


def webrtc_speech(vad_mode=2):
    """Return is_speech(frame) with webrtcvad, for int16 frames of 10, 20 or 30 ms."""
    vad = webrtcvad.Vad(vad_mode)
    return lambda frame: vad.is_speech(frame.tobytes(), SAMPLE_RATE)


class PhraseDetector:
    """Decides, frame by frame, where one phrase starts and ends."""

    def __init__(self, is_speech, frame_samples=FRAME_SAMPLES, noise_margin=6.0,
                 grace_period=1.0, max_phrase_secs=60.0, pre_record=0.5, background=()):
        """
        Create a detector for one phrase.

        is_speech(frame) is the voice detector (webrtc_speech() or Silero). background: the
        levels measured by a previous detector, to go on from them.
        """
        self.is_speech = is_speech
        self.frame_samples = frame_samples
        self.frame_secs = frame_samples / SAMPLE_RATE
        self.noise_margin = noise_margin
        self.max_frames = self.frames(max_phrase_secs) if max_phrase_secs > 0 else None
        self.background = collections.deque(background, maxlen=self.frames(BACKGROUND_SECS))
        self.start_window = collections.deque(maxlen=self.frames(START_SECS))
        self.end_window = collections.deque(maxlen=self.frames(grace_period))
        self.pre_buffer = collections.deque(
            maxlen=self.frames(pre_record) + self.frames(START_SECS))
        self.recorded = []
        self.background_db = float(np.median(self.background)) if self.background else None

    def frames(self, seconds):
        """Return how many frames last the given seconds (at least one)."""
        return max(1, int(round(seconds / self.frame_secs)))

    def is_voice(self, frame):
        """Return whether the frame is voice: the detector says so and it is loud enough."""
        level = level_db(frame)
        if not self.recorded:
            self.background.append(level)
            self.background_db = float(np.median(self.background))
        margin = self.noise_margin * (KEEP_MARGIN if self.recorded else 1.0)
        return level > self.background_db + margin and self.is_speech(frame)

    def add(self, frame):
        """Process one frame; return one of the events."""
        frame = frame.flatten()
        voice = self.is_voice(frame)

        if not self.recorded:
            self.pre_buffer.append(frame)
            self.start_window.append(voice)
            warmed_up = len(self.background) >= self.frames(WARMUP_SECS)
            if (warmed_up and len(self.start_window) == self.start_window.maxlen
                    and sum(self.start_window) >= START_RATIO * len(self.start_window)):
                self.recorded.extend(self.pre_buffer)
                self.end_window.extend([True] * self.end_window.maxlen)
                return STARTED
            return WAITING

        self.recorded.append(frame)
        self.end_window.append(voice)
        if sum(self.end_window) <= END_RATIO * len(self.end_window):
            return ENDED
        if self.max_frames and len(self.recorded) >= self.max_frames:
            return TOO_LONG
        return RECORDING

    def phrase(self):
        """Return the recorded audio, empty if no phrase started."""
        if not self.recorded:
            return np.array([])
        return np.concatenate(self.recorded)


class Transcription(threading.Thread):
    """Transcribes one phrase while the microphone keeps listening."""

    def __init__(self, transcribe, audio):
        """Start transcribing audio with transcribe(audio) in a new thread."""
        super().__init__(daemon=True)
        self.transcribe = transcribe
        self.audio = audio
        self.text = None
        self.error = None
        self.start()

    def run(self):
        """Keep the text, or the error."""
        try:
            self.text = self.transcribe(self.audio)
        except Exception as e:  # reported by listen()
            self.error = e


def listen(open_stream, new_detector, transcribe, max_wait, should_stop, on_status, logger):
    """
    Listen until a phrase is transcribed: the work of the Listen action.

    open_stream(): the microphone (or a file), read with read(n) -> (frames, overflow).
    new_detector(background): a PhraseDetector that goes on from the given levels.
    transcribe(audio): the text, or '' if it was not speech.
    Return (success, timed_out, text, message), as ListenActionServer expects.
    """
    deadline = time.time() + max_wait
    detector = new_detector(())
    pending = None  # transcription in progress

    def finish(result):
        if pending is not None:
            pending.join()  # the model is not used by two goals at once
        return result

    def outcome(done):
        """Return the result of a finished transcription, or None if it was not speech."""
        if done.error is not None:
            logger.error(f'Inference error: {done.error}')
            return False, False, '', str(done.error)
        if done.text:
            return True, False, done.text, done.text
        return None

    logger.info('Listening... (speak now)')
    on_status('listening')
    try:
        with open_stream() as stream:
            while True:
                if should_stop():
                    logger.info('Recording canceled.')
                    return finish((False, False, '', 'canceled'))

                frame, overflow = stream.read(detector.frame_samples)
                if overflow:
                    logger.warning('Audio overflow')
                event = detector.add(frame)

                if event == STARTED:
                    logger.info(f'Voice detected (background {detector.background_db:.0f} '
                                'dBFS), recording...')
                    on_status('speech_detected')
                elif event in (ENDED, TOO_LONG):
                    logger.info('End of the phrase.' if event == ENDED
                                else 'Longest phrase reached.')
                    if pending is not None:
                        pending.join()  # one transcription at a time
                        result = outcome(pending)
                        if result:
                            return result
                    pending = Transcription(transcribe, detector.phrase())
                    on_status('transcribing')
                    # Keep listening meanwhile, with the background already measured
                    detector = new_detector(detector.background)

                if pending is not None and not pending.is_alive():
                    done, pending = pending, None
                    result = outcome(done)
                    if result:
                        return result
                    # Not speech (see stt_filter): go on listening for the rest of max_wait
                    on_status('speech_detected' if detector.recorded else 'listening')

                if pending is None and not detector.recorded and time.time() > deadline:
                    logger.info(f'Timeout: nobody spoke (background '
                                f'{detector.background_db:.0f} dBFS).')
                    return False, True, '', 'Nobody spoke'
    except Exception as e:
        logger.error(f'Microphone error: {e}')
        return finish((False, False, '', str(e)))
