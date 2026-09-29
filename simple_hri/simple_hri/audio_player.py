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
Audio playback through an external command (aplay by default).

Replaces sound_play: no extra node is needed and the audio plays on the machine
where the node runs. Files are played one after another, never overlapped.
"""

import shlex
import shutil
import subprocess
import threading

# Tried in order when no command is given
DEFAULT_PLAYERS = ['aplay -q', 'pw-play', 'paplay']


class AudioPlayer:
    """Play WAV files with an external command, one after another."""

    def __init__(self, logger, command=''):
        """Choose the player command ('' = first available of DEFAULT_PLAYERS)."""
        self._logger = logger
        self._command = self._find_command(command)
        self._process = None
        self._watcher = None
        self._lock = threading.Lock()

        if self._command:
            logger.info(f'Audio player: {" ".join(self._command)}')
        else:
            logger.error('No audio player found: install alsa-utils (aplay) '
                         'or set the audio_player parameter')

    def _find_command(self, command):
        candidates = [command] if command else DEFAULT_PLAYERS
        for candidate in candidates:
            args = shlex.split(candidate)
            if args and shutil.which(args[0]):
                return args
        if command:
            self._logger.error(f'Audio player not found: {command}')
        return None

    def play(self, path):
        """
        Start playing a file and return without waiting for it to end.

        If another file is playing, wait for it to finish first. Return False if
        there is no player available.
        """
        with self._lock:
            self.wait()
            if self._command is None:
                return False
            self._process = subprocess.Popen(
                self._command + [path],
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            self._watcher = threading.Thread(
                target=self._watch, args=(self._process,), daemon=True)
            self._watcher.start()
            return True

    def _watch(self, process):
        """Wait for the process and report errors (a stopped playback is not an error)."""
        _, stderr = process.communicate()
        if process.returncode > 0:
            self._logger.error(
                f'{self._command[0]} failed ({process.returncode}): '
                f'{stderr.decode(errors="replace").strip()}')

    def is_playing(self):
        """Return True while a file is playing."""
        return self._watcher is not None and self._watcher.is_alive()

    def wait(self):
        """Block until the current playback ends."""
        watcher = self._watcher
        if watcher is not None:
            watcher.join()

    def stop(self):
        """Stop the current playback, if any."""
        process = self._process
        if process is not None and process.poll() is None:
            process.terminate()
