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
Find one phrase in the microphone audio with webrtcvad (stt_whisper and stt_openai).

webrtcvad alone takes room noise for voice: with a fan, a keyboard or people talking
far away it marks most 30 ms frames as voice, the phrase never gets 1 s of silence and the
recording never ends. Three rules make it robust:

  - A frame is voice only if webrtcvad says so AND it is noise_margin dB louder than the
    background, measured (median level of the last 3 s) while waiting for the person.
    The person speaks close to the microphone; the noise is the same before and after.
  - The phrase starts when most frames of the last 0.3 s are voice (one frame is not
    enough), and ends when few frames of the last grace_period seconds are voice
    (a click or a word of somebody else does not keep it open).
  - It never lasts more than max_phrase_secs.

Use: create one PhraseDetector per recording and give it every frame with add(); it says
when the phrase starts and ends, and phrase() returns the audio (with pre_record seconds
before the start, so that the first syllable is kept).
"""

import collections

import numpy as np
import webrtcvad

SAMPLE_RATE = 16000
FRAME_MS = 30  # webrtcvad accepts 10, 20 or 30 ms frames
FRAME_SAMPLES = SAMPLE_RATE * FRAME_MS // 1000

BACKGROUND_SECS = 3.0  # window that measures the background level while waiting
WARMUP_SECS = 0.5  # background measured before a phrase can start
START_SECS = 0.3  # window that starts the phrase...
START_RATIO = 0.7  # ...when this fraction of its frames are voice
END_RATIO = 0.2  # the phrase ends when at most this fraction of the last grace_period is voice

# Events returned by PhraseDetector.add()
WAITING = 'waiting'  # nobody speaks yet
STARTED = 'started'  # the phrase has just started
RECORDING = 'recording'  # inside the phrase
ENDED = 'ended'  # end of the phrase (silence)
TOO_LONG = 'too_long'  # end of the phrase (max_phrase_secs)


def frames(seconds):
    """Return how many frames last the given seconds (at least one)."""
    return max(1, int(round(seconds * 1000 / FRAME_MS)))


def level_db(frame):
    """Level of an int16 frame in dBFS (0 = full scale)."""
    samples = frame.astype(np.float32) / 32768.0
    return 20 * np.log10(np.sqrt(np.mean(samples ** 2)) + 1e-6)


class PhraseDetector:
    """Decides, frame by frame, where one phrase starts and ends."""

    def __init__(self, vad_mode=2, noise_margin=6.0, grace_period=1.0,
                 max_phrase_secs=15.0, pre_record=0.5):
        self.vad = webrtcvad.Vad(vad_mode)
        self.noise_margin = noise_margin
        self.max_frames = frames(max_phrase_secs)
        self.background = collections.deque(maxlen=frames(BACKGROUND_SECS))
        self.start_window = collections.deque(maxlen=frames(START_SECS))
        self.end_window = collections.deque(maxlen=frames(grace_period))
        self.pre_buffer = collections.deque(maxlen=frames(pre_record) + frames(START_SECS))
        self.recorded = []
        self.background_db = None

    def is_voice(self, frame):
        """Return whether the frame is voice: webrtcvad and louder than the background."""
        level = level_db(frame)
        if not self.recorded:
            # Only while waiting: the background must not include the phrase
            self.background.append(level)
            self.background_db = float(np.median(self.background))
        loud = level > self.background_db + self.noise_margin
        return loud and self.vad.is_speech(frame.tobytes(), SAMPLE_RATE)

    def add(self, frame):
        """Process one int16 frame of FRAME_SAMPLES samples; return one of the events."""
        frame = frame.flatten()
        voice = self.is_voice(frame)

        if not self.recorded:
            self.pre_buffer.append(frame)
            self.start_window.append(voice)
            warmed_up = len(self.background) >= frames(WARMUP_SECS)
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
        if len(self.recorded) >= self.max_frames:
            return TOO_LONG
        return RECORDING

    def phrase(self):
        """Return the recorded int16 audio, empty if no phrase started."""
        if not self.recorded:
            return np.array([], dtype=np.int16)
        return np.concatenate(self.recorded)
