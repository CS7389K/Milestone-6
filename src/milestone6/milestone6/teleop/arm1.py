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
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from milestone6.yolo.subscriber import YOLOSubscriber
from milestone6.yolo.yolo_data import YOLOData

from milestone6.teleop.publisher import TeleopPublisher
from milestone6.teleop.subscriber import TeleopSubscriber
from milestone6.teleop.constants import POSES
from milestone6.coco import COCO_CLASSES


class ArmMissionState(Enum):
    WAITING = 1
    REACHING = 2      # Extending arm to bottle
    GRIPPING = 3      # Closing gripper
    LIFTING = 4       # Lifting bottle
    HOLDING = 5       # Holding briefly
    MOVING = 6        # Moving forward
    PLACING = 7       # Lowering to place
    RELEASING = 8     # Opening gripper
    RETRACTING = 9    # Pulling arm back
    DONE = 10


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
        self.declare_parameter('target_bbox_width', 180)
        self.declare_parameter('bbox_tolerance', 15)

        self.target_class = int(self.get_parameter('target_class').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout_sec').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.center_tolerance = int(self.get_parameter('center_tolerance').value)

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.target_bbox_width = int(self.get_parameter('target_bbox_width').value)
        self.bbox_tolerance = int(self.get_parameter('bbox_tolerance').value)

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

        # Action tracking for trajectory completion
        self.trajectory_goal_handle = None
        self.trajectory_complete = False
        self.gripper_goal_handle = None
        self.gripper_complete = False
        
        # Timing for hold and move states
        self.hold_start_time = None
        self.move_start_time = None

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
            # Safe fallback if joint states are not ready - use home pose
            return POSES["home"].copy()

        # Object center
        obj_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)

        # Normalize to [-1, 1]
        norm_x = (obj_center_x - (self.image_width / 2.0)) / (self.image_width / 2.0)

        # Approx camera HFOV mapping for base rotation
        camera_hfov_rad = math.radians(62.0)
        desired_joint1 = norm_x * (camera_hfov_rad / 2.0)
        joint1 = max(-1.5, min(1.5, desired_joint1))

        # Use ROBOTIS-tested init pose for manipulation
        # This pose reaches ~22cm forward, ~18cm high
        # Expected bbox at this distance: ~180px for typical bottle
        reach_pose = POSES["init"].copy()
        reach_pose['joint1'] = joint1  # Adjust yaw to face object
        
        self.get_logger().info(
            f"Using ROBOTIS init pose (bbox={yolo_data.bbox_w:.0f}px): "
            f"j1={reach_pose['joint1']:.2f}, j2={reach_pose['joint2']:.2f}, "
            f"j3={reach_pose['joint3']:.2f}, j4={reach_pose['joint4']:.2f}"
        )
        
        return reach_pose

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------
    def _trajectory_done_callback(self, future):
        """Callback when arm trajectory action completes."""
        self.trajectory_goal_handle = future.result()
        if self.trajectory_goal_handle.accepted:
            result_future = self.trajectory_goal_handle.get_result_async()
            result_future.add_done_callback(self._trajectory_result_callback)
    
    def _trajectory_result_callback(self, future):
        """Callback when trajectory result is available."""
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.trajectory_complete = True
            self.get_logger().info("Trajectory completed successfully")
        else:
            self.get_logger().error(f"Trajectory failed with status: {result.status}")
            self.trajectory_complete = True  # Move on anyway
    
    def _gripper_done_callback(self, future):
        """Callback when gripper action completes."""
        self.gripper_goal_handle = future.result()
        if self.gripper_goal_handle.accepted:
            result_future = self.gripper_goal_handle.get_result_async()
            result_future.add_done_callback(self._gripper_result_callback)
    
    def _gripper_result_callback(self, future):
        """Callback when gripper result is available."""
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.gripper_complete = True
            self.get_logger().info("Gripper command completed")
        else:
            self.get_logger().error(f"Gripper failed with status: {result.status}")
            self.gripper_complete = True  # Move on anyway

    # ------------------------------------------------------------------
    # Helper methods for sending actions
    # ------------------------------------------------------------------
    def send_arm_action(self, pose_dict):
        """Send arm trajectory and wait for completion via callback."""
        self.trajectory_complete = False
        future = self.teleop_pub.arm_traj_ac.send_goal_async(
            self._create_trajectory_goal(pose_dict)
        )
        future.add_done_callback(self._trajectory_done_callback)
    
    def send_gripper_action(self, command_func):
        """Send gripper command and wait for completion via callback."""
        self.gripper_complete = False
        future = command_func()  # Returns future from gripper action
        if future:
            future.add_done_callback(self._gripper_done_callback)
        else:
            self.gripper_complete = True  # If no action client, mark as complete
    
    def _create_trajectory_goal(self, pose_dict):
        """Create FollowJointTrajectory goal from pose dictionary."""
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        point = JointTrajectoryPoint()
        point.positions = [
            pose_dict['joint1'],
            pose_dict['joint2'],
            pose_dict['joint3'],
            pose_dict['joint4']
        ]
        point.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points = [point]
        
        return goal

    # ------------------------------------------------------------------
    # State machine tick
    # ------------------------------------------------------------------
    def tick(self):

        # ------------------- WAITING -------------------
        if self.state == ArmMissionState.WAITING:
            if self.object_detected and self.last_yolo_data is not None and self.last_detection_time:
                elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
                if elapsed < self.detection_timeout:
                    # CHECK DISTANCE: Only start arm manipulation when at target distance
                    bbox_width = self.last_yolo_data.bbox_w
                    distance_error = abs(bbox_width - self.target_bbox_width)
                    
                    if distance_error <= self.bbox_tolerance:
                        self.get_logger().info(f"✓ At target distance (bbox={bbox_width:.0f}px). Starting manipulation...")
                        self.set_state(ArmMissionState.REACHING)
                    else:
                        # Still approaching - let base controller handle it
                        if bbox_width < self.target_bbox_width:
                            status = "TOO FAR - base moving forward"
                        else:
                            status = "TOO CLOSE - base moving backward"
                        self.get_logger().info(
                            f"Waiting for approach: bbox={bbox_width:.0f}px (target={self.target_bbox_width}±{self.bbox_tolerance}px) - {status}",
                            throttle_duration_sec=2.0
                        )
            return

        # ------------------- REACHING -------------------
        if self.state == ArmMissionState.REACHING:
            if not self.teleop_sub.have_joint_states:
                self.get_logger().warn("Waiting for joint states...")
                return

            if self.last_yolo_data is None:
                self.object_detected = False
                self.set_state(ArmMissionState.WAITING)
                return

            self.get_logger().info("="*60)
            self.get_logger().info("REACHING: Extending arm to bottle")
            self.get_logger().info("="*60)
            self.teleop_pub.set_velocity(0.0, 0.0)
            
            # Calculate and send arm pose (gripper already open from previous cycle)
            reach_pose = self.calculate_pose_from_object(self.last_yolo_data)
            
            # Send trajectory via action
            self.send_arm_action(reach_pose)
            self.set_state(ArmMissionState.GRIPPING)
            return

        # ------------------- GRIPPING -------------------
        if self.state == ArmMissionState.GRIPPING:
            # Wait for trajectory to complete
            if not self.trajectory_complete:
                return
            
            # Open gripper first (only after arm is in position)
            self.get_logger().info("Opening gripper at target position...")
            self.teleop_pub.gripper_open()
            
            # Small delay to ensure gripper is open, then close
            import time
            time.sleep(0.5)
            
            self.get_logger().info("GRIPPING: Closing gripper on bottle")
            self.teleop_pub.gripper_close()
            
            # Move to lifting immediately (gripper takes time to close)
            self.set_state(ArmMissionState.LIFTING)
            return

        # ------------------- LIFTING -------------------
        if self.state == ArmMissionState.LIFTING:
            if self.last_yolo_data is None:
                self.object_detected = False
                self.set_state(ArmMissionState.WAITING)
                return
            
            self.get_logger().info("LIFTING: Raising bottle")
            
            # Calculate lift pose - retract elbow while staying forward
            reach_pose = self.calculate_pose_from_object(self.last_yolo_data)
            lift_pose = {
                'joint1': reach_pose['joint1'],
                'joint2': reach_pose['joint2'],      # Keep same forward position
                'joint3': reach_pose['joint3'] + 0.3,  # Retract elbow to lift
                'joint4': 0.0
            }
            
            self.get_logger().info(
                f"Lift pose: j2={lift_pose['joint2']:.2f} (STAY FORWARD), "
                f"j3={lift_pose['joint3']:.2f} (RETRACT UP)"
            )
            
            self.send_arm_action(lift_pose)
            self.set_state(ArmMissionState.HOLDING)
            return

        # ------------------- HOLDING -------------------
        if self.state == ArmMissionState.HOLDING:
            # Wait for lift trajectory to complete
            if not self.trajectory_complete:
                return
            
            if self.hold_start_time is None:
                self.hold_start_time = self.get_clock().now()
                self.get_logger().info("HOLDING object briefly...")

            elapsed = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9
            if elapsed >= 1.5:
                self.hold_start_time = None
                self.set_state(ArmMissionState.MOVING)
            return

        # ------------------- MOVING -------------------
        if self.state == ArmMissionState.MOVING:
            if self.move_start_time is None:
                self.move_start_time = self.get_clock().now()
                self.get_logger().info("MOVING forward with object (0.5 feet / 15cm)...")
                self.teleop_pub.set_velocity(0.10, 0.0)
            
            elapsed = (self.get_clock().now() - self.move_start_time).nanoseconds / 1e9
            if elapsed >= 1.5:
                self.teleop_pub.set_velocity(0.0, 0.0)
                self.move_start_time = None
                self.get_logger().info("Reached destination. Placing object...")
                self.set_state(ArmMissionState.PLACING)
            return

        # ------------------- PLACING -------------------
        if self.state == ArmMissionState.PLACING:
            self.get_logger().info("PLACING: Lowering to placement position")
            self.teleop_pub.set_velocity(0.0, 0.0)

            # Lower to gripping position (center, no rotation)
            # Use ROBOTIS init pose for placement
            place_pose = POSES["init"].copy()
            place_pose['joint1'] = 0.0  # Center the base rotation
            
            self.get_logger().info(
                f"Place pose: j2={place_pose['joint2']:.2f}, j3={place_pose['joint3']:.2f}"
            )
            
            self.send_arm_action(place_pose)
            self.set_state(ArmMissionState.RELEASING)
            return

        # ------------------- RELEASING -------------------
        if self.state == ArmMissionState.RELEASING:
            # Wait for placement trajectory to complete
            if not self.trajectory_complete:
                return
            
            self.get_logger().info("RELEASING: Opening gripper")
            self.teleop_pub.gripper_open()
            
            self.set_state(ArmMissionState.RETRACTING)
            return

        # ------------------- RETRACTING -------------------
        if self.state == ArmMissionState.RETRACTING:
            self.get_logger().info("RETRACTING: Pulling arm back to safe home position")
            
            # Use ROBOTIS-tested home pose for safe retraction
            retract_pose = POSES["home"].copy()
            
            self.send_arm_action(retract_pose)
            self.set_state(ArmMissionState.DONE)
            return

        # ------------------- DONE -------------------
        if self.state == ArmMissionState.DONE:
            # Wait for retract to complete
            if not self.trajectory_complete:
                return
            
            self.get_logger().info("DONE: Mission complete. Resetting...")
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

