#!/usr/bin/env python3
"""
Part 4 Demo Script

This script demonstrates a complete Part 4 mission by sending a sequence
of natural language commands to the robot.

The scenario:
- Bottle is in front of the robot but off to the left
- Bear doll is approximately 2 meters ahead and slightly right
- Robot must find bottle, pick it up, navigate to bear, and place bottle

Usage:
    python3 demo_mission.py
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DemoMission(Node):
    """Demonstrate complete Part 4 mission."""

    def __init__(self):
        super().__init__('demo_mission')
        self.publisher = self.create_publisher(String, '/user_command', 10)

    def send_command(self, text: str, delay: float = 3.0):
        """Send a command and wait."""
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        print(f"\n[COMMAND] {text}")
        rclpy.spin_once(self, timeout_sec=0.1)

        if delay > 0:
            print(f"[WAITING] {delay} seconds for execution...")
            time.sleep(delay)


def main():
    rclpy.init()
    demo = DemoMission()

    print("\n" + "="*70)
    print("Part 4 Demo Mission")
    print("="*70)
    print("\nScenario:")
    print("  - Bottle is to the left of the robot")
    print("  - Bear doll is 2 meters ahead and slightly right")
    print("  - Mission: Find bottle, pick it up, transport to bear, place")
    print("="*70)

    input("\nPress Enter to start the mission...")

    try:
        # Phase 1: Locate and grab bottle
        print("\n" + "-"*70)
        print("PHASE 1: Locate and Grab Bottle")
        print("-"*70)

        demo.send_command("Scan the room to find the bottle", delay=8.0)
        demo.send_command("Turn left", delay=2.0)
        demo.send_command(
            "I can see the bottle, move forward a little", delay=1.5)
        demo.send_command("Pick up the bottle", delay=15.0)  # Grab takes time

        # Phase 2: Navigate toward bear location
        print("\n" + "-"*70)
        print("PHASE 2: Navigate to Bear Location")
        print("-"*70)

        demo.send_command(
            "The bear is about 2 meters straight ahead", delay=2.0)
        demo.send_command("Move forward", delay=1.5)
        demo.send_command("Move forward again", delay=1.5)
        demo.send_command("Now turn right a bit", delay=2.0)
        demo.send_command("Move forward", delay=1.5)

        # Phase 3: Find bear doll
        print("\n" + "-"*70)
        print("PHASE 3: Search for Bear Doll")
        print("-"*70)

        demo.send_command("Search for the bear doll", delay=2.0)
        demo.send_command("Turn left slowly to scan", delay=2.0)
        demo.send_command("Turn right", delay=2.0)

        # Phase 4: Place bottle
        print("\n" + "-"*70)
        print("PHASE 4: Place Bottle")
        print("-"*70)

        demo.send_command("Place the bottle in front of the bear", delay=12.0)
        demo.send_command("We are done with the mission", delay=2.0)

        print("\n" + "="*70)
        print("Mission Complete!")
        print("="*70 + "\n")

    except KeyboardInterrupt:
        print("\n\nMission interrupted by user")
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
