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
import time
from enum import Enum

import rclpy
from rclpy.node import Node

# YOLO detection
from milestone6.yolo.subscriber import YOLOSubscriber
from milestone6.yolo.yolo_data import YOLOData

# Arm control via teleop publisher
from milestone6.teleop.publisher import TeleopPublisher
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
        self.get_logger().info("Starting Teleop Publisher...")
        self.teleop_pub = TeleopPublisher(self)

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

        # Timing
        self.hold_start_time = None

        class_name = COCO_CLASSES.get(self.target_class, 'unknown')
        self.get_logger().info(f"Tracking COCO class: '{class_name}' (ID: {self.target_class})")
        self.get_logger().info("TeleopArm ready: Coordinated base+arm control")
        self.get_logger().info("Waiting for object detection...")


    # ------------------------------------------------------------------
    # YOLO callback
    # ------------------------------------------------------------------
    def _yolo_callback(self, data: YOLOData):
        """Process YOLO detections from YOLOSubscriber."""
        # Filter by target class
        if data.clz not in [self.target_class, 41]:
            self.get_logger().info(f"Ignoring detection: class {data.clz} != target {self.target_class}")
            return

        # Object detected!
        self.get_logger().info(
            f"TARGET DETECTED! class {data.clz}, "
            f"bbox=({data.bbox_x:.1f}, {data.bbox_y:.1f}, {data.bbox_w:.1f}x{data.bbox_h:.1f})"
        )
        self.object_detected = True
        self.last_detection_time = self.get_clock().now()
        self.last_yolo_data = data


    def set_state(self, new_state: ArmMissionState):
        """Transition to a new state."""
        if new_state != self.state:
            self.get_logger().info(f"State transition: {self.state.name} -> {new_state.name}")
            self.state = new_state


    def calculate_arm_position_from_object(self, yolo_data: YOLOData):
        """
        Calculate arm joint positions based on object location in camera frame.
        
        Strategy:
        - joint1 (base rotation): Turn arm left/right to track horizontal object position
        - joint2 (shoulder pitch): Adjust up/down based on vertical object position
        - joint3 (elbow pitch): Adjust reach based on distance (bbox size)
        - joint4 (wrist pitch): Complement joint2/joint3 to maintain gripper angle
        
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
        
        # Calculate distance estimate from bbox size
        bbox_ratio = yolo_data.bbox_w / self.image_width
        
        # Joint1: Horizontal rotation (yaw) to center object
        # Positive offset (right) -> rotate right (positive angle)
        joint1_target = offset_x * 1.5  # Scale to reasonable angle range
        
        # Joint2 (shoulder pitch): Vertical positioning
        # Base position around -0.5 to -0.9 (arm angled down for picking)
        # Positive offset_y (object lower) -> MORE negative (arm down more)
        # Negative offset_y (object higher) -> LESS negative (arm up more)
        joint2_base = -0.7  # Base downward angle
        joint2_vertical_adjust = offset_y * 0.3  # Adjust based on vertical position
        joint2_target = joint2_base - joint2_vertical_adjust
        
        # Joint3 (elbow pitch): Forward reach based on distance
        # Larger bbox (closer) -> less extension
        # Smaller bbox (farther) -> more extension
        # Range typically 0.2 to 0.6 radians
        joint3_min = 0.2  # Minimum extension (close)
        joint3_max = 0.6  # Maximum extension (far)
        # Invert bbox_ratio so smaller bbox = larger extension
        joint3_target = joint3_max - (bbox_ratio * (joint3_max - joint3_min) / 0.5)
        joint3_target = max(joint3_min, min(joint3_max, joint3_target))  # Clamp
        
        # Joint4 (wrist pitch): Compensate to keep gripper level
        # Should roughly equal -(joint2 + joint3) to maintain orientation
        joint4_target = -(joint2_target + joint3_target) + 0.2  # Small offset for slight downward angle
        
        self.get_logger().info(
            f"Arm calc: bbox_ratio={bbox_ratio:.3f}, offset_x={offset_x:.2f}, offset_y={offset_y:.2f}"
        )
        self.get_logger().info(
            f"  Targets: j1={joint1_target:.3f}, j2={joint2_target:.3f}, "
            f"j3={joint3_target:.3f}, j4={joint4_target:.3f}"
        )
        
        return {
            'joint1': joint1_target,
            'joint2': joint2_target,
            'joint3': joint3_target,
            'joint4': joint4_target
        }


    def tick(self):
        """Main state machine tick."""
        
        # ------------------- STATE: WAITING -------------------
        if self.state == ArmMissionState.WAITING:
            # Check if we have a fresh detection
            if self.object_detected and self.last_yolo_data is not None:
                # Check if detection is recent (within timeout)
                if self.last_detection_time:
                    elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
                    if elapsed < self.detection_timeout:
                        self.get_logger().info(f"Object detected (class {self.last_yolo_data.clz})! Preparing to grab...")
                        self.teleop_pub.set_velocity(linear_x=0.0, angular_z=0.0)
                        self.set_state(ArmMissionState.PREPARING)
                    else:
                        self.get_logger().info(f"Detection too old ({elapsed:.1f}s), waiting...")
            return

        # ------------------- STATE: PREPARING -------------------
        elif self.state == ArmMissionState.PREPARING:
            try:
                self.get_logger().info("PREPARING STATE: Opening gripper and positioning arm...")
                self.teleop_pub.set_velocity(linear_x=0.0, angular_z=0.0)
                
                if self.last_yolo_data is None:
                    self.get_logger().error("No YOLO data available, returning to WAITING")
                    self.set_state(ArmMissionState.WAITING)
                    return
                
                # Open gripper
                self.get_logger().info("Opening gripper...")
                self.teleop_pub.gripper_open()
                time.sleep(1.5)
                
                # Calculate and move to target position
                self.get_logger().info("Moving arm to track detected object...")
                target_positions = self.calculate_arm_position_from_object(self.last_yolo_data)
                self.get_logger().info(f"Target positions: {target_positions}")
                self.teleop_pub.send_arm_trajectory(target_positions)
                time.sleep(2.0)
                
                self.get_logger().info("Preparation complete! Starting grab...")
                self.set_state(ArmMissionState.GRABBING)
            except Exception as e:  # noqa: B902
                self.get_logger().error(f"Preparation failed: {e}")
                self.object_detected = False
                self.set_state(ArmMissionState.WAITING)
            return

        # ------------------- STATE: GRABBING -------------------
        elif self.state == ArmMissionState.GRABBING:
            try:
                self.get_logger().info("GRABBING STATE: Executing grab sequence...")
                
                if self.last_yolo_data is None:
                    self.get_logger().error("No YOLO data available, returning to WAITING")
                    self.object_detected = False
                    self.set_state(ArmMissionState.WAITING)
                    return
                
                # Recalculate positions for accuracy
                arm_positions = self.calculate_arm_position_from_object(self.last_yolo_data)
                
                # Position arm towards object
                self.get_logger().info("Positioning arm towards object...")
                self.teleop_pub.send_arm_trajectory(arm_positions)
                time.sleep(2.0)
                
                # Extend to grasp
                self.get_logger().info("Approaching for grasp...")
                grasp_positions = {
                    'joint1': arm_positions['joint1'],
                    'joint2': arm_positions['joint2'] - 0.2,
                    'joint3': arm_positions['joint3'] + 0.2,
                    'joint4': arm_positions['joint4'] + 0.1
                }
                self.teleop_pub.send_arm_trajectory(grasp_positions)
                time.sleep(2.0)
                
                # Close gripper
                self.get_logger().info("Closing gripper...")
                self.teleop_pub.gripper_close()
                time.sleep(2.0)
                
                # Lift object
                self.get_logger().info("Lifting object...")
                lift_positions = {
                    'joint1': arm_positions['joint1'],
                    'joint2': -0.4,
                    'joint3': 0.2,
                    'joint4': 0.8
                }
                self.teleop_pub.send_arm_trajectory(lift_positions)
                time.sleep(2.0)
                
                self.get_logger().info("Grab successful! Holding object...")
                self.set_state(ArmMissionState.HOLDING)
            except Exception as e:  # noqa: B902
                self.get_logger().error(f"Grab failed: {e}")
                self.object_detected = False
                self.set_state(ArmMissionState.WAITING)
            return

        # ------------------- STATE: HOLDING -------------------
        elif self.state == ArmMissionState.HOLDING:
            # Start holding timer if not set
            if self.hold_start_time is None:
                self.get_logger().info("Holding object for 2 seconds...")
                self.hold_start_time = self.get_clock().now()
            
            # Hold for 2 seconds, then release
            elapsed = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.hold_start_time = None
                self.get_logger().info("Releasing object...")
                self.set_state(ArmMissionState.RELEASING)
            return

        # ------------------- STATE: RELEASING -------------------
        elif self.state == ArmMissionState.RELEASING:
            try:
                self.get_logger().info("RELEASING STATE: Executing release sequence...")
                
                # Lower arm slightly
                self.get_logger().info("Lowering arm...")
                self.teleop_pub.send_arm_trajectory({
                    'joint1': 0.0,
                    'joint2': -0.7,
                    'joint3': 0.4,
                    'joint4': 0.9
                })
                time.sleep(2.0)
                
                # Open gripper
                self.get_logger().info("Opening gripper...")
                self.teleop_pub.gripper_open()
                time.sleep(2.0)
                
                # Retract arm
                self.get_logger().info("Retracting arm...")
                self.teleop_pub.send_arm_trajectory({
                    'joint1': 0.0,
                    'joint2': -0.3,
                    'joint3': 0.1,
                    'joint4': 0.6
                })
                time.sleep(2.0)
                
                self.get_logger().info("Release successful!")
                self.set_state(ArmMissionState.DONE)
            except Exception as e:  # noqa: B902
                self.get_logger().error(f"Release failed: {e}, but continuing to DONE")
                self.set_state(ArmMissionState.DONE)
            return

        # ------------------- STATE: DONE -------------------
        elif self.state == ArmMissionState.DONE:
            self.get_logger().info("Mission complete! Resetting to WAITING...")
            self.object_detected = False
            self.last_yolo_data = None
            self.last_detection_time = None
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
