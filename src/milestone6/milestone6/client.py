# Code taken from:
# https://github.com/ros2/demos/blob/1d01c8e3d06644c0d706ef83697a68efda7d0ad4/action_tutorials/action_tutorials_py/action_tutorials_py/fibonacci_action_client.py
#
#!/usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from .util import (
    record_audio_and_saveas,
    get_user_input,
    prompt_assistant
)
from interfaces.action import (
    EspeakAction,
    LlamaAction,
    WhisperAction
)


class ML5Client(Node):
    """
    Message Types: https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
    """
    def __init__(self):
        super().__init__('m5_client')
        self.declare_parameter('use_espeak', True)
        self.declare_parameter('use_llama', False)
        self.declare_parameter('use_whisper', True)

        self.use_espeak = self.get_parameter('use_espeak').value
        self.use_llama = self.get_parameter('use_llama').value
        self.use_whisper = self.get_parameter('use_whisper').value

        if self.use_espeak:
            self._espeak_client = ActionClient(self, EspeakAction, 'espeak_action')
        if self.use_llama:
            self._llama_client = ActionClient(self, LlamaAction, 'llama_action')
        if self.use_whisper:
            self._whisper_client = ActionClient(self, WhisperAction, 'whisper_action')

    def _request_blocking(
            self,
            client,
            ActionType,
            request_attr,
            request_msg
        ):
        request = ActionType.Goal()
        setattr(request, request_attr, request_msg)

        client.wait_for_server()
        send_goal_future = client.send_goal_async(
            request,
            feedback_callback=self._feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn(f"{str(type(ActionType))} request rejected")
            return None

        self.get_logger().info(f"{str(type(ActionType))} request accepted")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        return result

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))

    def __call__(
            self,
            client : str,
            request : str
    ):
        assert client in ['espeak', 'llama', 'whisper'], \
            f"Client must be one of 'espeak', 'llama', or 'whisper', got '{client}'"

        result = None
        if client == 'espeak' and self.use_espeak:
            result = self._request_blocking(
                self._espeak_client, EspeakAction, 'text', request
            ).result
        elif client == 'llama' and self.use_llama:
            result = self._request_blocking(
                self._llama_client, LlamaAction, 'prompt', request
            ).response
        elif client == 'whisper' and self.use_whisper:
            result = self._request_blocking(
                self._whisper_client, WhisperAction, 'file_name', request
            ).text

        return result


def main(args=None):
    rclpy.init(args=args)

    print("Initializing ML5 Client Node...")
    client = ML5Client()
    print("ML5 Client Node Initialized.")

    file_name = "voice_input.wav"
    while True:
        final_input = ""

        print("Prompt (or type 's' to detect voice): ")
        user_input = get_user_input()
        if user_input is None:
            break

        if user_input != "s":
            while user_input != "END":
                final_input += user_input + "\n"
                user_input = get_user_input()
        else:
            final_input = user_input

        if final_input == "s":
            # Record audio from microphone
            client("espeak", "speak in voice now")
            record_audio_and_saveas(file_name)
            # Transcribe audio file with whisper
            final_input = client("whisper", file_name)

        if client.use_llama:
            prompt_assistant(client, final_input)
        else:
            client("espeak", "Robot speaking.")
            client("espeak", final_input)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()