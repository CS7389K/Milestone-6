#!/usr/bin/env python3
"""
TeleopArm - Dynamic Object Tracking and Grabbing Node

This node uses YOLO object detection to dynamically control both the robot base
and arm to approach and grab objects (specifically bottles, class ID 39).
Arm movements are calculated based on object position in the camera frame.

Architecture:
    - YOLOSubscriber: Receives object detection data with position and size
    - TeleopPublisher: Controls both base (wheels) and arm movements
    - TeleopSubscriber: Monitors joint states for trajectory planning
    - TeleopArm (this node): Orchestrates coordinated base and arm control

States:
    WAITING    -> Waiting for object detection
    PREPARING  -> Object in range, preparing arm for grab
    GRABBING   -> Executing grab sequence
    HOLDING    -> Holding the grabbed object for 2 seconds
    RELEASING  -> Releasing the object
    DONE       -> Mission complete, resets to WAITING

Usage:
    ros2 run milestone6 part2_mission

Parameters:
    - target_class: COCO class ID to grab (default: 39 for bottles)
    - detection_timeout_sec: How long to wait for fresh detections (default: 1.0)
    - image_width: Camera image width (default: 500)
    - image_height: Camera image height (default: 320)
    - approach_bbox_threshold: BBox width to stop approaching (default: 150)
    - forward_speed: Base forward speed (default: 0.15)
    - turn_speed: Base turn speed (default: 1.5)

Example:
    ros2 run milestone6 part2_mission --ros-args -p target_class:=39
"""
import threading
import time
from enum import Enum

import rclpy
from rclpy.node import Node

# YOLO detection
from milestone6.yolo.subscriber import YOLOSubscriber
from milestone6.yolo.yolo_data import YOLOData

# Arm control via teleop publisher
from milestone6.teleop.publisher import TeleopPublisher
from milestone6.teleop.subscriber import TeleopSubscriber
from milestone6.coco import COCO_CLASSES


class ArmMissionState(Enum):
    """States for coordinated base and arm grabbing mission."""
    WAITING = 1           # Waiting for object detection
    PREPARING = 3         # Object in range, preparing arm for grab
    GRABBING = 4          # Executing grab sequence
    HOLDING = 5           # Object grabbed, holding
    RELEASING = 6         # Releasing the object
    DONE = 7             # Mission complete


class TeleopArm(Node):
    """
    Coordinated base and arm control node for object grabbing.
    Uses YOLO detection to approach and grab bottles dynamically.
    """
    def __init__(self):
        super().__init__('part2_mission')

        # ------------------- Parameters -------------------
        self.declare_parameter('target_class', 39)  # Default to bottle
        self.declare_parameter('detection_timeout_sec', 1.0)
        self.declare_parameter('image_width', 500)
        self.declare_parameter('image_height', 320)
        self.declare_parameter('approach_bbox_threshold', 150)  # Stop approaching when object is this wide
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_speed', 1.5)
        self.declare_parameter('center_tolerance', 50)  # Pixels tolerance for centering
        
        self.target_class = int(self.get_parameter('target_class').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout_sec').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.bbox_threshold = int(self.get_parameter('approach_bbox_threshold').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.center_tolerance = int(self.get_parameter('center_tolerance').value)

        # ------------------- YOLO Subscriber -------------------
        self.get_logger().info("Starting YOLO Subscriber...")
        self.yolo_subscriber = YOLOSubscriber(self, self._yolo_callback)

        # ------------------- Teleop control -------------------
        self.get_logger().info("Starting Teleop Subscriber and Publisher...")
        self.teleop_sub = TeleopSubscriber(self)
        self.teleop_pub = TeleopPublisher(self, self.teleop_sub)

        # ------------------- Timer -------------------
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz

        # Detection tracking
        self.last_detection_time = None
        self.last_yolo_data = None
        self.object_detected = False
        self.last_arm_command_time = None
        self.arm_command_interval = 0.5  # Only send arm commands every 0.5 seconds

        # State machine
        self.state = ArmMissionState.WAITING
        self.state_entered = True

        # Threading
        self.arm_action_complete = False
        self.arm_action_failed = False
        self.arm_thread = None
        self.hold_start_time = None

        class_name = COCO_CLASSES.get(self.target_class, 'unknown')
        self.get_logger().info("Part2Mission ready: Coordinated base+arm control")
        self.get_logger().info(f"Tracking COCO class: '{class_name}' (ID: {self.target_class})")
        self.get_logger().info("Waiting for object detection...")


    # ------------------------------------------------------------------
    # YOLO callback
    # ------------------------------------------------------------------
    def _yolo_callback(self, data: YOLOData):
        """Process YOLO detections from YOLOSubscriber."""
        self.get_logger().info(f"Received YOLO detection callback {str(data)}")

        # Filter by target class
        if data.clz != self.target_class:
            return

        # Object detected!
        self.object_detected = True
        self.last_detection_time = self.get_clock().now()
        self.last_yolo_data = data
        self.get_logger().debug(
            f"Bottle detected: class {data.clz}, "
            f"bbox=({data.bbox_x:.1f}, {data.bbox_y:.1f}, {data.bbox_w:.1f}x{data.bbox_h:.1f})"
        )


    # ------------------------------------------------------------------
    # State machine helpers
    # ------------------------------------------------------------------
    def set_state(self, new_state: ArmMissionState):
        """Transition to a new state."""
        self.get_logger().info(f"Requesting state transition to {new_state.name}")
        if new_state != self.state:
            self.get_logger().info(f"State transition: {self.state.name} -> {new_state.name}")
            self.state = new_state
            self.state_entered = True


    def is_detection_fresh(self):
        """Check if we've seen an object recently."""
        self.get_logger().info("Checking if detection is fresh")
        if not self.object_detected or self.last_detection_time is None:
            return False
        
        age = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        return age <= self.detection_timeout


    def calculate_arm_position_from_object(self, yolo_data: YOLOData):
        """
        Calculate arm joint positions based on object location in camera frame.
        
        Strategy:
        - joint1 (base rotation): Turn arm left/right to track horizontal object position
        - joint2, joint3, joint4: Adjust based on object distance (bbox size)
        
        Args:
            yolo_data: YOLO detection data
            
        Returns:
            dict: Target joint positions
        """
        # Calculate object center
        obj_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)
        obj_center_y = yolo_data.bbox_y + (yolo_data.bbox_h / 2.0)
        
        # Calculate horizontal offset from image center (-1.0 to 1.0)
        image_center_x = self.image_width / 2.0
        offset_x = (obj_center_x - image_center_x) / image_center_x
        
        # Calculate vertical offset from image center (-1.0 to 1.0)
        # Positive offset_y means object is LOWER in image (higher y coordinate)
        image_center_y = self.image_height / 2.0
        offset_y = (obj_center_y - image_center_y) / image_center_y
        
        # Map horizontal offset to joint1 rotation (-1.5 to 1.5 radians)
        # Positive offset (right) -> rotate right (positive angle)
        joint1_target = offset_x * 1.5
        
        # Map bbox size to arm extension
        # Small bbox (far) -> extend more, Large bbox (close) -> retract
        bbox_ratio = yolo_data.bbox_w / self.image_width
        
        if bbox_ratio < 0.15:  # Far away
            joint2_target = -0.6
            joint3_target = 0.3
            joint4_target = 0.9
        elif bbox_ratio < 0.3:  # Medium distance
            joint2_target = -0.7
            joint3_target = 0.4
            joint4_target = 1.0
        else:  # Close
            joint2_target = -0.8
            joint3_target = 0.5
            joint4_target = 1.0
        
        # Adjust joint2 based on vertical position
        # Convention: more negative joint2 = arm down, less negative = arm up
        # Positive offset_y (object lower in image) -> subtract to make more negative (arm down)
        # Negative offset_y (object higher in image) -> add (less subtraction) to make less negative (arm up)
        joint2_adjustment = -offset_y * 0.2
        joint2_target += joint2_adjustment
        
        self.get_logger().debug(
            f"Arm calc: offset_y={offset_y:.2f}, adjustment={joint2_adjustment:.3f}, "
            f"joint2={joint2_target:.3f}"
        )
        
        return {
            'joint1': joint1_target,
            'joint2': joint2_target,
            'joint3': joint3_target,
            'joint4': joint4_target
        }


    def control_base_towards_object(self, yolo_data: YOLOData):
        """
        Control robot base to approach the detected object.
        Similar to TeleopBase logic.
        
        Args:
            yolo_data: YOLO detection data
            
        Returns:
            bool: True if object is close enough (stopped moving)
        """
        # Calculate object center
        bbox_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)
        image_center_x = self.image_width / 2.0
        offset_x = bbox_center_x - image_center_x
        
        # Determine turning velocity
        if abs(offset_x) > self.center_tolerance:
            angular_ratio = -offset_x / (self.image_width / 2.0)
            angular_ratio = max(-1.0, min(1.0, angular_ratio))
            angular_vel = angular_ratio * self.turn_speed
        else:
            angular_vel = 0.0
        
        # Determine forward velocity based on bbox size
        if yolo_data.bbox_w < self.bbox_threshold:
            linear_vel = self.forward_speed
        else:
            linear_vel = 0.0  # Object is close enough, stop
        
        # Send velocity command
        self.teleop_pub.set_velocity(linear_x=linear_vel, angular_z=angular_vel)
        
        return linear_vel == 0.0  # Return True if we've stopped (close enough)


    # ------------------------------------------------------------------
    # Arm action threads
    # ------------------------------------------------------------------
    def _prepare_arm_thread(self):
        """Open gripper and move arm to ready position."""
        try:
            self.get_logger().info("Preparing arm for grab sequence...")
            
            # Wait for joint states to be available
            timeout = 5.0
            start = time.time()
            while not self.teleop_sub.have_joint_states:
                if time.time() - start > timeout:
                    raise RuntimeError("Timeout waiting for joint states")
                time.sleep(0.1)
            
            # Open gripper first
            self.get_logger().info("Opening gripper...")
            self.teleop_pub.gripper_open()
            time.sleep(1.5)
            
            # Move arm to ready position
            self.get_logger().info("Moving arm to ready position...")
            target_positions = {
                'joint1': 0.0,
                'joint2': -0.3,
                'joint3': -0.4,
                'joint4': 0.0
            }
            self.teleop_pub.send_arm_trajectory(target_positions)
            time.sleep(2.0)
            
            self.arm_action_complete = True
            self.arm_action_failed = False
            self.get_logger().info("Arm preparation complete!")
        except Exception as e:
            self.get_logger().error(f"Arm preparation failed: {e}")
            self.arm_action_complete = True
            self.arm_action_failed = True


    def _grab_thread(self):
        """Execute complete grab sequence."""
        try:
            self.get_logger().info("Starting GRAB sequence...")
            
            # Step 1: Pre-grasp position
            self.get_logger().info("Moving to pre-grasp...")
            self.teleop_pub.send_arm_trajectory({
                'joint1': 0.0,
                'joint2': -0.6,
                'joint3': 0.3,
                'joint4': 0.9
            })
            time.sleep(2.0)
            
            # Step 2: Approach grasp
            self.get_logger().info("Approaching object...")
            self.teleop_pub.send_arm_trajectory({
                'joint1': 0.0,
                'joint2': -0.8,
                'joint3': 0.5,
                'joint4': 1.0
            })
            time.sleep(2.0)
            
            # Step 3: Close gripper
            self.get_logger().info("Closing gripper...")
            self.teleop_pub.gripper_close()
            time.sleep(2.0)
            
            # Step 4: Lift
            self.get_logger().info("Lifting object...")
            self.teleop_pub.send_arm_trajectory({
                'joint1': 0.0,
                'joint2': -0.4,
                'joint3': 0.2,
                'joint4': 0.8
            })
            time.sleep(2.0)
            
            self.arm_action_complete = True
            self.arm_action_failed = False
            self.get_logger().info("GRAB sequence completed successfully!")
        except Exception as e:
            self.get_logger().error(f"Grab failed: {e}")
            self.arm_action_complete = True
            self.arm_action_failed = True


    def _release_thread(self):
        """Execute release sequence."""
        try:
            self.get_logger().info("Starting RELEASE sequence...")
            
            # Step 1: Lower slightly
            self.get_logger().info("Lowering arm...")
            self.teleop_pub.send_arm_trajectory({
                'joint1': 0.0,
                'joint2': -0.7,
                'joint3': 0.4,
                'joint4': 0.9
            })
            time.sleep(2.0)
            
            # Step 2: Open gripper
            self.get_logger().info("Opening gripper...")
            self.teleop_pub.gripper_open()
            time.sleep(2.0)
            
            # Step 3: Retract
            self.get_logger().info("Retracting arm...")
            self.teleop_pub.send_arm_trajectory({
                'joint1': 0.0,
                'joint2': -0.3,
                'joint3': 0.1,
                'joint4': 0.6
            })
            time.sleep(2.0)
            
            self.arm_action_complete = True
            self.arm_action_failed = False
            self.get_logger().info("RELEASE sequence completed successfully!")
        except Exception as e:
            self.get_logger().error(f"Release failed: {e}")
            self.arm_action_complete = True
            self.arm_action_failed = True


    def start_arm_action(self, thread_func):
        """Start an arm action in a separate thread."""
        self.arm_action_complete = False
        self.arm_action_failed = False
        self.arm_thread = threading.Thread(target=thread_func, daemon=True)
        self.arm_thread.start()


    # ------------------------------------------------------------------
    # Main tick (state machine)
    # ------------------------------------------------------------------
    def tick(self):
        """Main state machine tick."""
        
        # ------------------- STATE: WAITING -------------------
        if self.state == ArmMissionState.WAITING:
            if self.state_entered:
                self.get_logger().info("Waiting for object detection...")
                self.state_entered = False
                # Stop any movement
                self.teleop_pub.set_velocity(linear_x=0.0, angular_z=0.0)
            
            # Check if we have a fresh detection
            if self.is_detection_fresh():
                self.get_logger().info("Object detected! Preparing to grab...")
                self.set_state(ArmMissionState.PREPARING)
            return

        # ------------------- STATE: PREPARING -------------------
        if self.state == ArmMissionState.PREPARING:
            if self.state_entered:
                self.state_entered = False
                # Ensure base is stopped
                self.teleop_pub.set_velocity(linear_x=0.0, angular_z=0.0)
                self.start_arm_action(self._prepare_arm_thread)
                return
            
            # Wait for preparation to complete
            if self.arm_action_complete:
                if self.arm_action_failed:
                    self.get_logger().error("Arm preparation failed, returning to WAITING")
                    self.object_detected = False
                    self.set_state(ArmMissionState.WAITING)
                else:
                    self.get_logger().info("Preparation complete! Starting grab...")
                    self.set_state(ArmMissionState.GRABBING)
            return

        # ------------------- STATE: GRABBING -------------------
        if self.state == ArmMissionState.GRABBING:
            if self.state_entered:
                self.state_entered = False
                self.start_arm_action(self._grab_thread)
                return
            
            # Wait for grab to complete
            if self.arm_action_complete:
                if self.arm_action_failed:
                    self.get_logger().error("Grab failed, returning to WAITING")
                    self.object_detected = False
                    self.set_state(ArmMissionState.WAITING)
                else:
                    self.get_logger().info("Grab successful! Holding object...")
                    self.set_state(ArmMissionState.HOLDING)
            return

        # ------------------- STATE: HOLDING -------------------
        if self.state == ArmMissionState.HOLDING:
            if self.state_entered:
                self.get_logger().info("Holding object for 2 seconds...")
                self.state_entered = False
                self.hold_start_time = self.get_clock().now()
            
            # Hold for 2 seconds, then release
            elapsed = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.get_logger().info("Releasing object...")
                self.set_state(ArmMissionState.RELEASING)
            return

        # ------------------- STATE: RELEASING -------------------
        if self.state == ArmMissionState.RELEASING:
            if self.state_entered:
                self.state_entered = False
                self.start_arm_action(self._release_thread)
                return
            
            # Wait for release to complete
            if self.arm_action_complete:
                if self.arm_action_failed:
                    self.get_logger().error("Release failed, but continuing to DONE")
                else:
                    self.get_logger().info("Release successful!")
                self.set_state(ArmMissionState.DONE)
            return

        # ------------------- STATE: DONE -------------------
        if self.state == ArmMissionState.DONE:
            if self.state_entered:
                self.get_logger().info("Mission complete! Resetting to WAITING...")
                self.state_entered = False
                self.object_detected = False
                time.sleep(1.0)
                self.set_state(ArmMissionState.WAITING)
            return


def main():
    rclpy.init()
    node = TeleopArm()
    try:
        while rclpy.ok():
            node.tick()
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
