#!/usr/bin/env python3
"""
arm2.py

FixedArmPose - Simple validation node for OpenManipulatorX

Purpose:
- Verify that arm trajectory action and gripper action are working.
- Provide a deterministic, "known-good" pose sequence.

This node does NOT use YOLO.
It is meant to be a quick hardware sanity check.

Sequence:
1) Open gripper
2) Move to a validated "home/reach" pose
3) Close gripper (optional)
4) Move to a safe "retract" pose
5) Open gripper
6) Exit (or keep alive if you prefer)

Usage (after entry_points):
    ros2 run milestone6 arm2
"""

import time

import rclpy
from rclpy.node import Node

from milestone6.teleop.publisher import TeleopPublisher
from milestone6.teleop.subscriber import TeleopSubscriber


class FixedArmPose(Node):
    def __init__(self):
        super().__init__('teleop_arm2_fixed_pose')

        self.teleop_pub = TeleopPublisher(self)
        self.teleop_sub = TeleopSubscriber(self)

        # One-shot timer to run after node starts
        self._ran = False
        self.timer = self.create_timer(0.2, self.run_once)

        self.get_logger().info("FixedArmPose node ready.")

    def run_once(self):
        """Execute the fixed pose sequence once."""
        if self._ran:
            return

        # Wait briefly for joint states and action servers
        if not self.teleop_sub.have_joint_states:
            self.get_logger().warn("Waiting for joint states before running fixed pose...")
            return

        self._ran = True

        # Validated pose targets (known-good style values)
        # You can tweak these slightly if your hardware calibration differs.
        reach_pose = {
            'joint1': 0.0,
            'joint2': -1.05,
            'joint3': 0.35,
            'joint4': 0.70
        }

        retract_pose = {
            'joint1': 0.0,
            'joint2': -0.30,
            'joint3': 0.10,
            'joint4': 0.60
        }

        try:
            self.get_logger().info("Step 1: Open gripper.")
            self.teleop_pub.gripper_open()
            time.sleep(1.0)

            self.get_logger().info("Step 2: Move to reach pose.")
            self.teleop_pub.send_arm_trajectory(reach_pose)
            time.sleep(2.0)

            self.get_logger().info("Step 3: Close gripper (verification).")
            self.teleop_pub.gripper_close()
            time.sleep(1.0)

            self.get_logger().info("Step 4: Move to retract pose.")
            self.teleop_pub.send_arm_trajectory(retract_pose)
            time.sleep(1.5)

            self.get_logger().info("Step 5: Open gripper.")
            self.teleop_pub.gripper_open()
            time.sleep(1.0)

            self.get_logger().info("Fixed pose sequence complete.")
        except Exception as e:
            self.get_logger().error(f"Fixed pose sequence failed: {e}")

        # Optional: shutdown after completing once
        self.get_logger().info("Shutting down FixedArmPose node.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FixedArmPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

