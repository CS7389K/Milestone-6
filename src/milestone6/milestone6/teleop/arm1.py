#!/usr/bin/env python3
"""
arm1.py

TeleopArmV1 - Robust Object Grabbing Node (Pose-based)

This version intentionally avoids complicated IK from pixels.
Instead, it uses validated OpenMANIPULATOR-X joint presets for
joint2-4, and only adjusts joint1 (base yaw) to face the target.

Goal:
- Improve real-hardware success rate by using "known-good" poses.

Architecture:
    - YOLOSubscriber: receives target detections (bbox + class)
    - TeleopPublisher: sends base velocity + arm trajectory + gripper commands
    - TeleopSubscriber: reads joint states for safer sequencing

States:
    WAITING    -> wait for fresh detection
    PREPARING  -> open gripper + move to reaching pose
    GRABBING   -> small reach + close gripper + lift
    HOLDING    -> hold for a short time
    RELEASING  -> place down + open gripper + retract
    DONE       -> reset

Usage (after you add entry_points if needed):
    ros2 run milestone6 arm1
or via launch if you wire it in.

Parameters:
    target_class: COCO class id to grab (default: 39 bottle)
    detection_timeout_sec: detection freshness window
    image_width / image_height: camera size used for centering
    center_tolerance: pixel tolerance for joint1 alignment
"""

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node

from milestone6.yolo.subscriber import YOLOSubscriber
from milestone6.yolo.yolo_data import YOLOData

from milestone6.teleop.publisher import TeleopPublisher
from milestone6.teleop.subscriber import TeleopSubscriber
from milestone6.coco import COCO_CLASSES


class ArmMissionState(Enum):
    WAITING = 1
    PREPARING = 3
    GRABBING = 4
    HOLDING = 5
    RELEASING = 6
    DONE = 7


class TeleopArmV1(Node):
    """
    Pose-based grabbing mission.

    Design choices:
    - joint2-4 are fixed to validated values to avoid unreachable commands.
    - joint1 is computed from target horizontal offset only.
    """

    def __init__(self):
        super().__init__('teleop_arm1')

        # ------------------- Parameters -------------------
        self.declare_parameter('target_class', 39)
        self.declare_parameter('detection_timeout_sec', 1.0)
        self.declare_parameter('image_width', 500)
        self.declare_parameter('image_height', 320)
        self.declare_parameter('center_tolerance', 50)

        # base approach params (optional; you can keep base control in base.py)
        self.declare_parameter('forward_speed', 0.0)
        self.declare_parameter('turn_speed', 0.0)

        self.target_class = int(self.get_parameter('target_class').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout_sec').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.center_tolerance = int(self.get_parameter('center_tolerance').value)

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)

        # ------------------- YOLO Subscriber -------------------
        self.get_logger().info("Starting YOLO Subscriber...")
        self.yolo_subscriber = YOLOSubscriber(self, self._yolo_callback)

        # ------------------- Teleop control -------------------
        self.get_logger().info("Starting Teleop Publisher...")
        self.teleop_pub = TeleopPublisher(self)

        # ------------------- Joint State Tracking -------------------
        self.get_logger().info("Starting Teleop Subscriber (joint states)...")
        self.teleop_sub = TeleopSubscriber(self)

        # ------------------- Timer -------------------
        self.timer = self.create_timer(0.1, self.tick)

        # Detection tracking
        self.last_detection_time = None
        self.last_yolo_data = None
        self.object_detected = False

        # State machine
        self.state = ArmMissionState.WAITING

        # Timing
        self.hold_start_time = None

        class_name = COCO_CLASSES.get(self.target_class, 'unknown')
        self.get_logger().info(f"TeleopArmV1 tracking class '{class_name}' (ID: {self.target_class})")
        self.get_logger().info("TeleopArmV1 ready. Waiting for object detection...")

    # ------------------------------------------------------------------
    # YOLO callback
    # ------------------------------------------------------------------
    def _yolo_callback(self, data: YOLOData):
        """Store latest valid detection of the target class."""
        # Some models may confuse bottle/cup; allow a tiny whitelist if you want.
        if data.clz != self.target_class:
            return

        self.object_detected = True
        self.last_detection_time = self.get_clock().now()
        self.last_yolo_data = data

    def set_state(self, new_state: ArmMissionState):
        if new_state != self.state:
            self.get_logger().info(f"State: {self.state.name} -> {new_state.name}")
            self.state = new_state

    # ------------------------------------------------------------------
    # Core pose logic
    # ------------------------------------------------------------------
    def calculate_pose_from_object(self, yolo_data: YOLOData):
        """
        Compute pose using validated joint2-4 values.

        joint1 is adjusted from horizontal pixel offset.
        """

        if not self.teleop_sub.have_joint_states:
            # Safe fallback if joint states are not ready
            return {
                'joint1': 0.0,
                'joint2': -1.05,
                'joint3': 0.35,
                'joint4': 0.70
            }

        # Object center
        obj_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)
        obj_center_y = yolo_data.bbox_y + (yolo_data.bbox_h / 2.0)

        # Normalize to [-1, 1]
        norm_x = (obj_center_x - (self.image_width / 2.0)) / (self.image_width / 2.0)
        norm_y = (obj_center_y - (self.image_height / 2.0)) / (self.image_height / 2.0)

        # Approx camera HFOV mapping for base rotation
        camera_hfov_rad = math.radians(62.0)
        desired_joint1 = norm_x * (camera_hfov_rad / 2.0)
        joint1 = max(-1.5, min(1.5, desired_joint1))

        # FORWARD REACHING POSE (positive joint2 = down/forward, negative joint3 = extend)
        # Adjust based on bbox size (larger = closer)
        bbox_width_normalized = yolo_data.bbox_w / self.image_width
        
        if bbox_width_normalized > 0.35:  # Very close
            joint2 = 0.3
            joint3 = -0.2
            joint4 = 0.0
            self.get_logger().info(f"VERY CLOSE (bbox={yolo_data.bbox_w:.0f}px)")
        elif bbox_width_normalized > 0.25:  # Close
            joint2 = 0.5
            joint3 = -0.3
            joint4 = 0.0
            self.get_logger().info(f"CLOSE (bbox={yolo_data.bbox_w:.0f}px)")
        else:  # Far
            joint2 = 0.7
            joint3 = -0.4
            joint4 = 0.0
            self.get_logger().info(f"FAR (bbox={yolo_data.bbox_w:.0f}px)")
        
        # Adjust for vertical position
        if norm_y > 0.3:  # Object low in frame
            joint2 += 0.2
            self.get_logger().info(f"Object LOW in frame, reaching down more")

        return {
            'joint1': joint1,
            'joint2': joint2,
            'joint3': joint3,
            'joint4': joint4
        }

    # ------------------------------------------------------------------
    # State machine tick
    # ------------------------------------------------------------------
    def tick(self):

        # ------------------- WAITING -------------------
        if self.state == ArmMissionState.WAITING:
            if self.object_detected and self.last_yolo_data is not None and self.last_detection_time:
                elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
                if elapsed < self.detection_timeout:
                    self.set_state(ArmMissionState.PREPARING)
            return

        # ------------------- PREPARING -------------------
        if self.state == ArmMissionState.PREPARING:
            try:
                if not self.teleop_sub.have_joint_states:
                    self.get_logger().warn("Waiting for joint states...")
                    return

                self.get_logger().info("="*60)
                self.get_logger().info("PREPARING: opening gripper and positioning arm")
                self.get_logger().info("="*60)
                self.teleop_pub.set_velocity(0.0, 0.0)

                # Open gripper
                self.teleop_pub.gripper_open()
                time.sleep(1.5)

                if self.last_yolo_data is None:
                    self.object_detected = False
                    self.set_state(ArmMissionState.WAITING)
                    return

                # Move to reach pose
                reach_pose = self.calculate_pose_from_object(self.last_yolo_data)
                self.get_logger().info(
                    f"Moving to reach pose: j1={reach_pose['joint1']:.2f}, "
                    f"j2={reach_pose['joint2']:.2f}, j3={reach_pose['joint3']:.2f}, j4={reach_pose['joint4']:.2f}"
                )
                self.get_logger().info(
                    f"Object at ({self.last_yolo_data.bbox_x:.0f}, {self.last_yolo_data.bbox_y:.0f}), "
                    f"size {self.last_yolo_data.bbox_w:.0f}x{self.last_yolo_data.bbox_h:.0f}px"
                )
                self.teleop_pub.send_arm_trajectory(reach_pose)
                time.sleep(2.5)  # More time to reach position

                self.set_state(ArmMissionState.GRABBING)
            except Exception as e:
                self.get_logger().error(f"PREPARING failed: {e}")
                self.object_detected = False
                self.set_state(ArmMissionState.WAITING)
            return

        # ------------------- GRABBING -------------------
        if self.state == ArmMissionState.GRABBING:
            try:
                self.get_logger().info("GRABBING: approach + close + lift.")

                if self.last_yolo_data is None:
                    self.object_detected = False
                    self.set_state(ArmMissionState.WAITING)
                    return

                # CRITICAL: Use CURRENT detection, not stale data
                reach_pose = self.calculate_pose_from_object(self.last_yolo_data)

                # Approach: Extend further to reach the object
                # Key insight: Need to reach DOWN more (increase joint2) and FORWARD more (decrease joint3)
                grasp_pose = {
                    'joint1': reach_pose['joint1'],
                    'joint2': reach_pose['joint2'] + 0.2,  # Reach down/forward MORE
                    'joint3': reach_pose['joint3'] - 0.2,  # Extend elbow MORE (negative = extend)
                    'joint4': 0.0  # Keep level
                }
                self.get_logger().info(
                    f"Approaching bottle: j1={grasp_pose['joint1']:.2f}, "
                    f"j2={grasp_pose['joint2']:.2f} (DOWN), j3={grasp_pose['joint3']:.2f} (EXTEND)"
                )
                self.teleop_pub.send_arm_trajectory(grasp_pose)
                time.sleep(2.5)  # Give time to reach

                # Close gripper
                self.get_logger().info("Closing gripper on bottle...")
                self.teleop_pub.gripper_close()
                time.sleep(2.0)

                # Lift pose - STAY FORWARD, just retract elbow to lift
                # DO NOT reduce joint2, that makes it go backward!
                lift_pose = {
                    'joint1': grasp_pose['joint1'],  # Keep same rotation
                    'joint2': grasp_pose['joint2'],  # KEEP same forward position!
                    'joint3': grasp_pose['joint3'] + 0.3,  # Retract elbow to lift (positive = retract/up)
                    'joint4': 0.0
                }
                self.get_logger().info(
                    f"Lifting bottle: j2={lift_pose['joint2']:.2f} (STAY FORWARD), "
                    f"j3={lift_pose['joint3']:.2f} (RETRACT UP)"
                )
                self.teleop_pub.send_arm_trajectory(lift_pose)
                time.sleep(2.0)

                self.set_state(ArmMissionState.HOLDING)
            except Exception as e:
                self.get_logger().error(f"GRABBING failed: {e}")
                self.object_detected = False
                self.set_state(ArmMissionState.WAITING)
            return

        # ------------------- HOLDING -------------------
        if self.state == ArmMissionState.HOLDING:
            if self.hold_start_time is None:
                self.hold_start_time = self.get_clock().now()
                self.get_logger().info("HOLDING object...")

            elapsed = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9
            if elapsed >= 1.5:
                self.hold_start_time = None
                self.set_state(ArmMissionState.RELEASING)
            return

        # ------------------- RELEASING -------------------
        if self.state == ArmMissionState.RELEASING:
            try:
                self.get_logger().info("RELEASING: lower + open + retract.")

                lower_pose = {
                    'joint1': 0.0,
                    'joint2': -0.80,
                    'joint3': 0.60,
                    'joint4': 0.80
                }
                self.teleop_pub.send_arm_trajectory(lower_pose)
                time.sleep(1.5)

                self.teleop_pub.gripper_open()
                time.sleep(1.0)

                retract_pose = {
                    'joint1': 0.0,
                    'joint2': -0.30,
                    'joint3': 0.10,
                    'joint4': 0.60
                }
                self.teleop_pub.send_arm_trajectory(retract_pose)
                time.sleep(1.5)

                self.set_state(ArmMissionState.DONE)
            except Exception as e:
                self.get_logger().error(f"RELEASING failed: {e}")
                self.set_state(ArmMissionState.DONE)
            return

        # ------------------- DONE -------------------
        if self.state == ArmMissionState.DONE:
            self.get_logger().info("DONE: reset to WAITING.")
            self.object_detected = False
            self.last_yolo_data = None
            self.last_detection_time = None
            self.set_state(ArmMissionState.WAITING)
            return


def main(args=None):
    rclpy.init(args=args)
    node = TeleopArmV1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

