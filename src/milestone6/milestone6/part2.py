#!/usr/bin/env python3
"""
Part 2: Complete Object Grab and Transport Sequence

This node implements the full Part 2 requirements:
1. Detect bottle using YOLO
2. Center on the bottle (rotate base to align)
3. Approach the bottle (move forward until close enough)
4. Grab the bottle using arm + gripper
5. Move forward while gripping the bottle (transport)
6. Release the bottle safely

States:
    IDLE          -> Waiting for object detection
    CENTERING     -> Rotating base to center object in frame
    APPROACHING   -> Moving forward to optimal grab distance
    GRABBING      -> Executing arm grab sequence
    TRANSPORTING  -> Moving forward with object
    RELEASING     -> Placing object and retracting arm
    DONE          -> Complete

Usage:
    ros2 run milestone6 part2
"""

from enum import Enum

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient

from milestone6.robot import Robot
from milestone6.teleop.subscriber import TeleopSubscriber


class State(Enum):
    """States for complete grab and transport sequence."""
    IDLE = 1            # Waiting for object detection
    CENTERING = 2       # Rotating base to center object in frame
    APPROACHING = 3     # Moving forward to optimal grab distance
    GRABBING = 4        # Executing arm grab sequence
    TRANSPORTING = 5    # Moving forward with object
    RELEASING = 6       # Placing object and retracting arm
    DONE = 7            # Complete

class Part2(Robot):
    """
    Complete Part 2 implementation with object centering,
    grabbing using FollowJointTrajectory, transport, and release.

    Extends Part1 to add state machine for grab-transport-release sequence.
    """

    PARAMETERS = {
        'detection_timeout': 1.0,       # seconds (override Part1's 0.5s)
        'transport_distance': 1.0,      # meters
        # Arm joint position parameters (radians)
        'grab_joint2': 0.5,             # Forward reach
        'grab_joint3': -0.3,            # Extend elbow
        'grab_joint4': 0.0,             # Level gripper
        'grab_vertical_adjust': 0.2,    # Extra reach for low objects
        'grasp_extension': 0.2,         # Extra extension to grasp
        # Lift positions
        'lift_joint2': -0.5,            # Lift up
        'lift_joint3': 0.4,             # Retract
        'lift_joint4': 0.6,             # Adjust wrist
        # Lower positions
        'lower_joint1': 0.0,            # Center position
        'lower_joint2': 0.3,            # Lower down
        'lower_joint3': -0.2,           # Extend
        'lower_joint4': 0.0,            # Level
        # Home positions
        'home_joint1': 0.0,
        'home_joint2': -1.05,
        'home_joint3': 0.35,
        'home_joint4': 0.70,
        # Gripper position parameters
        'gripper_open': 0.025,          # Open position
        'gripper_close': -0.025,        # Close position
    }

    def __init__(self):
        super().__init__('part2')

        self.info("Initializing Part 2: Complete Grab and Transport Sequence...")
        self.info(f"Parameters: {self.params}")

        # ------------------- Action Clients -------------------
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd'
        )

        # ------------------- Joint State Subscriber -------------------
        self.info("Starting Joint State Subscriber...")
        self.teleop_sub = TeleopSubscriber(self)

        # ------------------- State Machine -------------------
        self.state = State.IDLE

        # Sub-state tracking for multi-step operations
        self.grab_step = 0
        self.release_step = 0
        self.step_start_time = None

        # Transport tracking
        self.transport_start_time = None
        self.transport_duration = None  # Calculated from distance/speed

        # Action future tracking
        self.arm_action_future = None
        self.gripper_action_future = None

        self.info("Part 2 initialized and ready.")

    def _tick(self):
        # ----------------------------------- IDLE ----------------------------------- #
        if self.state == State.IDLE:
            if self.detection_is_fresh() and self.last_yolo_data is not None:
                self.info("\n" + "=" * 70)
                self.info("START: Object detected!")
                self.info(
                    f"Current state: {self.state}, changing to CENTERING")
                self.info("=" * 70)
                self.state = State.CENTERING
            else:
                return

        # --------------------------------- CENTERING -------------------------------- #
        if self.state == State.CENTERING:
            self.debug("In CENTERING state, checking detection freshness...")
            if not self.detection_is_fresh():
                self.warning(
                    "Lost detection during centering, returning to IDLE")
                self.stop_movement()
                self.state = State.IDLE
                return

            self.debug(
                "[CENTERING] Attempting to center on object...")
            if self.center_on_object(self.last_yolo_data):
                self.info(
                    "[CENTERING] Object centered! Moving to APPROACHING...")
                self.state = State.APPROACHING

        # -------------------------------- APPROACHING ------------------------------- #
        if self.state == State.APPROACHING:
            if not self.detection_is_fresh():
                self.warning(
                    "Lost detection during approach, returning to IDLE")
                self.stop_movement()
                self.state = State.IDLE
                return

            # First ensure centered while approaching
            if not self.is_centered(self.last_yolo_data):
                self.center_on_object(self.last_yolo_data)
                return

            # Then approach to correct distance
            if self.approach_object(self.last_yolo_data):
                self.info(
                    "[APPROACHING] At grab distance! Starting GRABBING sequence...")
                self.stop_movement()
                self.grab_step = 0
                self.state = State.GRABBING

        # --------------------------------- GRABBING --------------------------------- #
        if self.state == State.GRABBING:
            self.execute_grab_sequence()

        # ------------------------------- TRANSPORTING ------------------------------- #
        if self.state == State.TRANSPORTING:
            if self.transport_start_time is None:
                # Calculate how long to move forward
                self.transport_duration = self.transport_distance / self.speed
                self.transport_start_time = self.get_clock().now()

                self.info(f"\n{'=' * 70}")
                self.info(f"[TRANSPORTING] Moving forward {self.transport_distance}m "
                          f"(~{self.transport_duration:.1f}s)")
                self.info(f"{'=' * 70}")

                # Start moving forward
                self.teleop_publisher.set_velocity(
                    linear_x=self.speed, angular_z=0.0)
            else:
                # Check if we've moved far enough
                elapsed = (self.get_clock().now() -
                           self.transport_start_time).nanoseconds / 1e9

                if elapsed >= self.transport_duration:
                    self.stop_movement()
                    self.info(f"[TRANSPORTING] Traveled {self.transport_distance}m! "
                              f"Starting RELEASING sequence...")
                    self.release_step = 0
                    self.state = State.RELEASING
                else:
                    remaining = self.transport_duration - elapsed
                    if int(remaining) != int(remaining + 0.1):  # Log every second
                        self.info(
                            f"Transporting... {remaining:.0f}s remaining")

        # --------------------------------- RELEASING -------------------------------- #
        if self.state == State.RELEASING:
            self.execute_release_sequence()

        # ----------------------------------- DONE ----------------------------------- #
        if self.state == State.DONE:
            self.info("\n" + "=" * 70)
            self.info("COMPLETE!")
            self.info("Object successfully grabbed and transported 1 meter!")
            self.info("=" * 70 + "\n")

            # Reset for next
            self.last_yolo_data = None
            self.last_detection_time = None
            self.transport_start_time = None
            self.state = State.IDLE
            return

    # ---------------------------------------------------------------------------- #
    #                                Helper Methods                                #
    # ---------------------------------------------------------------------------- #
    def execute_grab_sequence(self):
        """Multi-step grabbing sequence using FollowJointTrajectory."""

        # Step 0: Open gripper
        if self.grab_step == 0:
            self.info("[GRAB] Step 0: Opening gripper...")
            self.gripper_action_future = self.send_gripper_command(
                self.gripper_open)
            self.step_start_time = self.get_clock().now()
            self.grab_step = 1

        # Step 1: Wait for gripper to open
        elif self.grab_step == 1:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.info(
                    "[GRAB] Step 1: Moving arm to grab position...")
                grab_pos = self.calculate_grab_position(self.last_yolo_data)
                self.arm_action_future = self.send_arm_trajectory(
                    grab_pos, time_sec=2.5)
                self.step_start_time = self.get_clock().now()
                self.grab_step = 2

        # Step 2: Wait for arm to position
        elif self.grab_step == 2:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 3.0:  # Give extra time for trajectory
                self.info("[GRAB] Step 2: Extending to grasp...")
                # Extend slightly more to grasp
                current = self.teleop_sub.joint_positions
                grasp_pos = {
                    'joint1': current.get('joint1', 0.0),
                    'joint2': current.get('joint2', self.grab_joint2),
                    'joint3': min(1.5, current.get('joint3', self.grab_joint3) + self.grasp_extension),
                    'joint4': current.get('joint4', self.grab_joint4)
                }
                self.arm_action_future = self.send_arm_trajectory(
                    grasp_pos, time_sec=1.5)
                self.step_start_time = self.get_clock().now()
                self.grab_step = 3

        # Step 3: Wait for extension, then close gripper
        elif self.grab_step == 3:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.info("[GRAB] Step 3: Closing gripper...")
                self.gripper_action_future = self.send_gripper_command(
                    self.gripper_close)
                self.step_start_time = self.get_clock().now()
                self.grab_step = 4

        # Step 4: Wait for gripper, then lift
        elif self.grab_step == 4:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.info("[GRAB] Step 4: Lifting object...")
                lift_pos = {
                    'joint1': self.teleop_sub.joint_positions.get('joint1', 0.0),
                    'joint2': self.lift_joint2,
                    'joint3': self.lift_joint3,
                    'joint4': self.lift_joint4
                }
                self.arm_action_future = self.send_arm_trajectory(
                    lift_pos, time_sec=2.0)
                self.step_start_time = self.get_clock().now()
                self.grab_step = 5

        # Step 5: Wait for lift to complete
        elif self.grab_step == 5:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.5:
                self.info(
                    "[GRAB] Object grabbed! Moving to TRANSPORTING...")
                self.grab_step = 0
                self.state = State.TRANSPORTING

    def execute_release_sequence(self):
        """Multi-step release sequence."""

        # Step 0: Lower arm
        if self.release_step == 0:
            self.info(
                "[RELEASE] Step 0: Lowering arm to place object...")
            lower_pos = {
                'joint1': self.lower_joint1,
                'joint2': self.lower_joint2,
                'joint3': self.lower_joint3,
                'joint4': self.lower_joint4
            }
            self.arm_action_future = self.send_arm_trajectory(
                lower_pos, time_sec=2.0)
            self.step_start_time = self.get_clock().now()
            self.release_step = 1

        # Step 1: Wait for lowering, then open gripper
        elif self.release_step == 1:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.5:
                self.info(
                    "[RELEASE] Step 1: Opening gripper to release...")
                self.gripper_action_future = self.send_gripper_command(
                    self.gripper_open)
                self.step_start_time = self.get_clock().now()
                self.release_step = 2

        # Step 2: Wait for gripper, then retract arm
        elif self.release_step == 2:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.info(
                    "[RELEASE] Step 2: Retracting arm to home position...")
                home_pos = {
                    'joint1': self.home_joint1,
                    'joint2': self.home_joint2,
                    'joint3': self.home_joint3,
                    'joint4': self.home_joint4
                }
                self.arm_action_future = self.send_arm_trajectory(
                    home_pos, time_sec=2.0)
                self.step_start_time = self.get_clock().now()
                self.release_step = 3

        # Step 3: Wait for retraction to complete
        elif self.release_step == 3:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.5:
                self.info("[RELEASE] Release complete!")
                self.release_step = 0
                self.state = State.DONE


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Part2()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in Part 2: {e}")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
