#!/usr/bin/env python3
"""
Espeak Subscriber Node

This node subscribes to /text_to_speech topic and uses the espeak backend
to speak the received text messages.

Topic:
    /text_to_speech (std_msgs/String) - Text to be spoken

Usage:
    ros2 run milestone6 espeak
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from milestone6.nlp.backends.espeak import EspeakBackend


class EspeakSubscriber(Node):
    """ROS2 node that subscribes to text and speaks it using espeak."""

    def __init__(self):
        super().__init__('espeak_subscriber')

        # Declare parameters
        self.declare_parameter('speech_rate', 140)

        # Get parameters
        speech_rate = self.get_parameter('speech_rate').value

        # Initialize espeak backend
        self.espeak = EspeakBackend(speech_rate=speech_rate)

        # Subscribe to text_to_speech topic
        self.subscription = self.create_subscription(
            String,
            '/text_to_speech',
            self._text_callback,
            10
        )

        self.get_logger().info(
            f"Espeak Subscriber initialized (speech_rate: {speech_rate})")
        self.get_logger().info("Listening for text on /text_to_speech...")

    def _text_callback(self, msg: String):
        """Callback to speak received text."""
        text = msg.data
        self.get_logger().info(f"[ESPEAK] Speaking: '{text}'")

        try:
            success = self.espeak(text)
            if not success:
                self.get_logger().error(f"Failed to speak: '{text}'")
        except Exception as e:
            self.get_logger().error(f"Error speaking text: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EspeakSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in espeak_subscriber: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
