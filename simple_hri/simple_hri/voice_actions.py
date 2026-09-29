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
Listen and Say action servers shared by the STT and TTS nodes.

The nodes keep their original services (/stt_service, /tts_service) unchanged.
These actions run the same code and add what a service cannot offer:
feedback, cancellation and, for Say, finishing when playback really ends.
"""

import time
import wave

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from simple_hri_interfaces.action import Listen, Say

FEEDBACK_PERIOD = 0.1  # s


def estimate_duration(text):
    """Estimate how long it takes to say a text (~10 chars per second + margin)."""
    return len(text) * 0.1 + 0.5


def audio_file_duration(path):
    """Return the duration of a WAV file in seconds, or None if it cannot be read."""
    try:
        with wave.open(path, 'rb') as audio:
            return audio.getnframes() / audio.getframerate()
    except Exception:
        return None


class ListenActionServer:
    """
    Listen action on top of a listen function.

    listen_fn(max_wait, should_stop, on_status) -> (success, timed_out, text, message)
    must check should_stop() while recording and call on_status() when the stage changes.
    """

    def __init__(self, node, listen_fn, lock, name='stt_action'):
        """Create the action server on the node."""
        self._node = node
        self._listen_fn = listen_fn
        self._lock = lock  # Shared with the service: only one recording at a time
        self._server = ActionServer(
            node, Listen, name,
            execute_callback=self._execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def _execute(self, goal_handle):
        feedback = Listen.Feedback()

        def on_status(status):
            feedback.status = status
            goal_handle.publish_feedback(feedback)

        with self._lock:
            success, timed_out, text, message = self._listen_fn(
                goal_handle.request.max_wait,
                lambda: goal_handle.is_cancel_requested,
                on_status)

        result = Listen.Result()
        result.success = success
        result.timed_out = timed_out
        result.text = text
        result.message = message

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self._node.get_logger().info('Listen action canceled')
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


class SayActionServer:
    """
    Say action on top of a speak function.

    speak_fn(text) -> (success, duration, message) synthesizes the text and starts playing it
    without blocking; duration is the playback length in seconds.
    stop_fn() stops the playback.
    """

    def __init__(self, node, speak_fn, stop_fn, lock, name='tts_action'):
        """Create the action server on the node."""
        self._node = node
        self._speak_fn = speak_fn
        self._stop_fn = stop_fn
        self._lock = lock  # Shared with the service: only one synthesis at a time
        self._server = ActionServer(
            node, Say, name,
            execute_callback=self._execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def _execute(self, goal_handle):
        result = Say.Result()

        with self._lock:
            success, duration, message = self._speak_fn(goal_handle.request.text)

        if not success:
            result.success = False
            result.message = message
            goal_handle.abort()
            return result

        # Wait for the playback to end, reporting the remaining time and attending cancellation
        feedback = Say.Feedback()
        end_time = time.time() + duration
        while time.time() < end_time:
            if goal_handle.is_cancel_requested:
                self._stop_fn()
                result.success = False
                result.message = 'canceled'
                goal_handle.canceled()
                self._node.get_logger().info('Say action canceled: playback stopped')
                return result
            feedback.remaining = max(0.0, end_time - time.time())
            goal_handle.publish_feedback(feedback)
            time.sleep(FEEDBACK_PERIOD)

        result.success = True
        result.message = message
        goal_handle.succeed()
        return result


def spin_multithreaded(node):
    """
    Spin a node so that an action can be canceled while it is being executed.

    With the default single-threaded spin, the cancel request would wait until the
    action finishes.
    """
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
