# Code taken from:
# https://github.com/ros2/demos/blob/1d01c8e3d06644c0d706ef83697a68efda7d0ad4/action_tutorials/action_tutorials_py/action_tutorials_py/fibonacci_action_server.py
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
# limitations under the License.import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from interfaces.action import (
    EspeakAction,
    LlamaAction,
    WhisperAction
)
from .backends import (
    EspeakBackend,
    LlamaBackend,
    WhisperBackend
)

class ML5Server(Node):
    """
    Message Types: https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
    """
    def __init__(self):
        super().__init__('m5_server')
        self.declare_parameter('use_espeak', False)
        self.declare_parameter('use_llama', False)
        self.declare_parameter('use_whisper', False)
        self.declare_parameter('llama_instruct', False)
        self.declare_parameter('llama_model_path', "")

        self.use_espeak = self.get_parameter('use_espeak').value
        self.use_llama = self.get_parameter('use_llama').value
        self.use_whisper = self.get_parameter('use_whisper').value
        self.llama_instruct = self.get_parameter('llama_instruct').value
        self.llama_model_path = self.get_parameter('llama_model_path').value

        if not self.use_espeak and not self.use_whisper and not self.use_whisper:
            self.get_logger().warn(
                "No backends are enabled. Please enable at least one backend "
                "by setting the appropriate parameters."
            )

        if not self.use_llama and self.llama_instruct:
            self.get_logger().warn(
                "Parameter 'llama_instruct' is set to True but 'use_llama' is False. "
                "Llama instruct mode will be ignored because llama is not in use."
            )

        if self.use_llama and self.use_whisper:
            self.get_logger().warn(
                "Both 'use_llama' and 'use_whisper' are set to True. "
                "This may lead to high GPU memory usage depending on the model sizes."
            )

        # Start all of the action servers
        if self.use_espeak:
            self._espeak = EspeakBackend()
            self._espeak_server = ActionServer(
                self,
                EspeakAction,
                'espeak_action',
                self._callback_espeak
            )
        if self.use_llama:
            if self.llama_model_path != "":
                self._llama = LlamaBackend(model_path=self.llama_model_path)
            else:
                self._llama = LlamaBackend(instruct=self.llama_instruct)
            self._llama_server = ActionServer(
                self,
                LlamaAction,
                'llama_action',
                self._callback_llama
            )
        if self.use_whisper:
            device = "cuda" if not self.use_llama else "cpu"
            self._whisper = WhisperBackend(device=device)
            self._whisper_server = ActionServer(
                self,
                WhisperAction,
                'whisper_action',
                self._callback_whisper
            )

    def _call_backend(
            self,
            ctx,
            backend,
            request_attr,
            result_attr,
            feedback_attr,
            ResultType,
            FeedbackType
        ):
        error = ""
        try:
            backend_result = backend(getattr(ctx.request, request_attr))
            ctx.succeed()
        except Exception as e:
            error = str(e)
            # ctx.failed()

        feedback = FeedbackType()
        setattr(feedback, feedback_attr, error)
        feedback.feedback = error
        ctx.publish_feedback(feedback)

        result = ResultType()
        setattr(result, result_attr, backend_result)

        self.get_logger().info(f"Completed {str(type(backend))} request.")
        self.get_logger().info(f"Result: {backend_result}")

        return result

    def _callback_espeak(self, ctx):
        self.get_logger().info(f"Received espeak request: text={ctx.request.text}")
        if not self.use_espeak:
            ctx.failed()
            return EspeakAction.Result()
        return self._call_backend(
            ctx,
            self._espeak,
            "text",
            "result",
            "feedback",
            EspeakAction.Result,
            EspeakAction.Feedback
        )

    def _callback_llama(self, ctx):
        self.get_logger().info(f"Received llama request: prompt={ctx.request.prompt}")
        if not self.use_llama:
            ctx.failed()
            return LlamaAction.Result()
        return self._call_backend(
            ctx,
            self._llama,
            "prompt",
            "response",
            "feedback",
            LlamaAction.Result,
            LlamaAction.Feedback
        )

    def _callback_whisper(self, ctx):
        self.get_logger().info(f"Received whisper request: file_name={ctx.request.file_name}")
        if not self.use_whisper:
            ctx.failed()
            return WhisperAction.Result()
        return self._call_backend(
            ctx,
            self._whisper,
            "file_name",
            "text",
            "feedback",
            WhisperAction.Result,
            WhisperAction.Feedback
        )


def main(args=None):
    rclpy.init(args=args)

    print("Initializing ML5 Server Node...")
    server = ML5Server()
    print("ML5 Server Node Initialized.")

    rclpy.spin(server)

    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()