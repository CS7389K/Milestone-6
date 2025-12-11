aaaaa#!/usr/bin/env python3
"""
Interactive Command Interface for Part 4

This script provides an interactive terminal interface for sending
natural language commands to the robot.

Usage:
    python3 send_command.py
    
Then type commands interactively, or use command line:
    python3 send_command.py "Turn around and look for the bottle"
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CommandSender(Node):
    """Simple node to send user commands."""

    def __init__(self):
        super().__init__('command_sender')
        self.publisher = self.create_publisher(String, '/user_command', 10)

    def send_command(self, text: str):
        """Send a command to the robot."""
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent: {text}")


def interactive_mode(sender):
    """Run interactive command prompt."""
    print("\n" + "="*60)
    print("Part 4 Interactive Command Interface")
    print("="*60)
    print("\nExample commands:")
    print("  - Scan the room for the bottle")
    print("  - Turn left")
    print("  - Move forward")
    print("  - Pick up the bottle")
    print("  - Search for the bear")
    print("  - Place the bottle")
    print("  - We are done")
    print("\nType 'quit' or 'exit' to stop")
    print("="*60 + "\n")

    try:
        while rclpy.ok():
            try:
                command = input("Command> ").strip()

                if not command:
                    continue

                if command.lower() in ['quit', 'exit', 'q']:
                    print("Exiting...")
                    break

                sender.send_command(command)

                # Give time for message to be published
                rclpy.spin_once(sender, timeout_sec=0.1)

            except EOFError:
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user")


def main(args=None):
    rclpy.init(args=args)
    sender = CommandSender()

    # Check if command provided as argument
    if len(sys.argv) > 1:
        # Send command from command line
        command = ' '.join(sys.argv[1:])
        sender.send_command(command)
        rclpy.spin_once(sender, timeout_sec=0.5)
    else:
        # Run interactive mode
        interactive_mode(sender)

    sender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
