#!/usr/bin/env python3
"""
LLaMA Publisher Node

This node provides an LLM-based interface that takes user text input,
processes it through a LLaMA model with a system prompt for high-level
planning, and publishes atomic robot actions.

The node subscribes to /user_command (text input from user) and publishes
to /llm_action (atomic robot actions).

Topics:
    /user_command (std_msgs/String) - Natural language commands from user
    /llm_action (std_msgs/String) - Atomic actions for robot execution

Usage:
    ros2 run milestone6 llama_publisher
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from milestone6.nlp.backends.llama import LlamaBackend
from milestone6.nlp.prompts import SYSTEM_PROMPT


class LlamaPublisher(Node):
    """ROS2 node that uses LLaMA to translate user commands to robot actions."""

    # System prompt for high-level planning

    def __init__(self):
        super().__init__('llama_publisher')

        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('instruct', True)
        self.declare_parameter('n_ctx', 512)
        self.declare_parameter('n_threads', 4)
        self.declare_parameter('n_gpu_layers', 33)
        self.declare_parameter('temperature', 0.1)
        self.declare_parameter('top_p', 0.95)
        self.declare_parameter('max_tokens', 32)

        # Get parameters
        model_path = self.get_parameter('model_path').value
        instruct = self.get_parameter('instruct').value
        n_ctx = self.get_parameter('n_ctx').value
        n_threads = self.get_parameter('n_threads').value
        n_gpu_layers = self.get_parameter('n_gpu_layers').value
        self.temperature = self.get_parameter('temperature').value
        self.top_p = self.get_parameter('top_p').value
        self.max_tokens = self.get_parameter('max_tokens').value

        # Initialize LLaMA backend
        self.get_logger().info("Loading LLaMA model...")
        self.llama = LlamaBackend(
            model_path=model_path,
            instruct=instruct,
            n_ctx=n_ctx,
            n_threads=n_threads,
            n_gpu_layers=n_gpu_layers
        )
        self.get_logger().info("LLaMA model loaded successfully")

        # Create publisher for atomic actions
        self.action_publisher = self.create_publisher(
            String,
            '/llm_action',
            10
        )

        # Subscribe to user commands
        self.command_subscription = self.create_subscription(
            String,
            '/user_command',
            self._command_callback,
            10
        )

        self.get_logger().info("LLaMA Publisher initialized")
        self.get_logger().info("Listening for commands on /user_command...")
        self.get_logger().info("Publishing actions to /llm_action...")

    def _command_callback(self, msg: String):
        """Process user command through LLaMA and publish atomic action."""
        user_text = msg.data.strip()

        if not user_text:
            return

        self.get_logger().info(f"[USER] {user_text}")

        # Build prompt with system context and conversation history
        prompt = self._build_prompt(user_text)

        try:
            # Get LLM response
            self.get_logger().info("Processing with LLaMA...")
            self.get_logger().info(f"Prompt: {repr(prompt)}")
            response = self.llama(
                prompt,
                max_tokens=self.max_tokens,
                temperature=self.temperature,
                top_p=self.top_p,
                stream=True,
                stop=["\n\n", "User:", "[/INST]", "</s>",
                      "[INST]", "\n1.", "\n2.", "\n-"]
            )

            # Log the full response for debugging
            self.get_logger().info(f"LLM raw response: {repr(response)}")
            self.get_logger().info(f"Response length: {len(response)} chars")

            # Extract atomic action from response
            action = self._extract_action(response)

            if action:
                self.get_logger().info(f"[LLM] {action}")

                # Publish action
                action_msg = String()
                action_msg.data = action
                self.action_publisher.publish(action_msg)
            else:
                self.get_logger().warn(
                    f"Could not extract action from: {response}")

        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")

    def _build_prompt(self, user_text: str) -> str:
        """Build prompt with system context and user input."""
        # For instruct models, use [INST] format
        # For chat models, use conversational format
        instruct = self.get_parameter('instruct').value

        if instruct:
            # Llama-2-32K-Instruct format: [INST]\n{system + user}\n[/INST]\n
            # Combine system prompt and user text
            combined = f"{SYSTEM_PROMPT}\n\n{user_text}"
            prompt = f"[INST]\n{combined}\n[/INST]\n"
        else:
            # Chat format
            prompt = SYSTEM_PROMPT + "\n\n"
            prompt += f"User: {user_text}\n"
            prompt += "Assistant: "

        return prompt

    def _extract_action(self, response: str) -> str:
        """Extract atomic action from LLM response."""
        # Clean up response
        response = response.strip()

        # Log the raw response for debugging
        self.get_logger().debug(f"Raw LLM response: {repr(response)}")

        # Try to find action anywhere in the response (not just at start of line)
        response_upper = response.upper()

        # First, try to find parameterized actions (they need special handling)
        if 'SEARCH' in response_upper:
            # Extract SEARCH <object> pattern
            import re
            match = re.search(r'SEARCH\s+(\w+)', response_upper)
            if match:
                return f"SEARCH {match.group(1)}"
            else:
                return "SEARCH"

        if 'TRANSPORT_TO' in response_upper:
            # Extract TRANSPORT_TO <object> pattern
            import re
            match = re.search(r'TRANSPORT_TO\s+(\w+)', response_upper)
            if match:
                return f"TRANSPORT_TO {match.group(1)}"
            else:
                return "TRANSPORT_TO"

        # Check for simple actions (order matters - check longer ones first)
        for action in ['MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT', 'SCAN',
                       'GRAB', 'PLACE', 'DONE']:
            if action in response_upper:
                return action

        # If no valid action found, log and return empty
        self.get_logger().warn(
            f"No valid action found in response: {repr(response)}")
        return ""


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LlamaPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in llama_publisher: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
