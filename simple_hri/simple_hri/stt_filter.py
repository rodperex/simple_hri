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
Discard what Whisper "hears" when nobody speaks.

With noise or silence, Whisper often invents a sentence: the subtitle credits and
YouTube phrases it saw in training ("¡Suscríbete!", "Subtítulos por la comunidad de
Amara.org"). Measured with Whisper small (openai-whisper and faster-whisper, CPU) on
room-like noise and on speech: in noise every invented segment had a no-speech
probability of 0.78-0.87; in speech, at most 0.38. Two checks:

  keep_segment(no_speech_prob): drop the segments that are probably not speech
  is_hallucination(text): the final text is empty, only punctuation, or a known
                          hallucination (it also catches a short "Sí." that Whisper
                          turned into "¡Suscríbete!")

A discarded transcription is treated as "nobody spoke" (timed_out in the Listen action).
"""

import re
import unicodedata

# Segments above this no-speech probability are dropped. Whisper's own rule also requires
# avg_logprob < -1.0, which the invented sentences (-0.75 to -1.0) do not meet
NO_SPEECH_PROB = 0.6

# Known Whisper hallucinations, normalized with normalize()
HALLUCINATIONS = {
    # Spanish
    'suscribete',
    'suscribete al canal',
    'no olvides suscribirte',
    'subtitulos por la comunidad de amaraorg',
    'subtitulos realizados por la comunidad de amaraorg',
    'subtitulado por la comunidad de amaraorg',
    'subtitulos en espanol de la comunidad de amaraorg',
    'amaraorg',
    'gracias por ver',
    'gracias por ver el video',
    'gracias por vernos',
    # English
    'subscribe',
    'please subscribe',
    'thank you for watching',
    'thanks for watching',
    'subtitles by the amaraorg community',
}


def normalize(text):
    """Lowercase, without accents, punctuation or repeated spaces."""
    text = unicodedata.normalize('NFD', text.lower())
    text = ''.join(c for c in text if unicodedata.category(c) != 'Mn')
    text = re.sub(r'[^\w\s]', '', text)
    return ' '.join(text.split())


def keep_segment(no_speech_prob):
    """Whether a segment is probably speech."""
    return no_speech_prob <= NO_SPEECH_PROB


def is_hallucination(text):
    """Whether a final transcription is not something a person said."""
    normalized = normalize(text)
    return not normalized or normalized in HALLUCINATIONS
