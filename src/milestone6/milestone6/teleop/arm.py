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
import math
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

        # ------------------- OpenMANIPULATOR-X Link Lengths (in meters) -------------------
        # Based on ROBOTIS OpenMANIPULATOR-X specifications
        # Total reach: 380mm (0.38m)
        self.L1 = 0.077  # Base to joint 2 (height of link1)
        self.L2 = 0.130  # Joint 2 to joint 3 (shoulder to elbow, link2 length)
        self.L3 = 0.124  # Joint 3 to joint 4 (elbow to wrist, link3 length)
        self.L4 = 0.126  # Joint 4 to gripper (wrist to end effector, link4+link5)
        
        self.get_logger().info(
            f"Arm link lengths: L1={self.L1}m, L2={self.L2}m, L3={self.L3}m, L4={self.L4}m"
        )

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
        
        # Manipulator frame reference - store initial detection for coordinate frame
        self.manipulator_reference_frame = None  # Store first detection as reference
        self.using_reference_frame = False

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
        
        # Store first detection as manipulator reference frame
        if self.manipulator_reference_frame is None:
            self.manipulator_reference_frame = data
            self.get_logger().info(
                f"REFERENCE FRAME SET: Using detection at "
                f"({data.bbox_x:.1f}, {data.bbox_y:.1f}) as manipulator coordinate origin"
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
        Calculate arm joint positions using inverse kinematics for OpenMANIPULATOR-X.
        
        Uses proper IK with link lengths to reach objects detected by YOLO.
        
        OpenMANIPULATOR-X Kinematics:
            - Link lengths: L1=77mm, L2=130mm, L3=124mm, L4=126mm (total reach 380mm)
            - joint1: Base rotation (yaw) around Z-axis
            - joint2: Shoulder pitch
            - joint3: Elbow pitch  
            - joint4: Wrist pitch
        
        Camera-to-Target Transform:
            - Camera FOV and bbox position estimate 3D target location
            - Assume camera mounted ~150mm forward, ~200mm up from base
        """
        # Calculate object center in image coordinates
        obj_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)
        obj_center_y = yolo_data.bbox_y + (yolo_data.bbox_h / 2.0)
        
        # Normalize to [-1, 1] range
        norm_x = (obj_center_x - (self.image_width / 2.0)) / (self.image_width / 2.0)
        norm_y = (obj_center_y - (self.image_height / 2.0)) / (self.image_height / 2.0)
        
        # ==================== ESTIMATE TARGET 3D POSITION ====================
        # Camera is mounted forward and up from base
        # Estimate distance from bbox size (larger = closer)
        bbox_width_ratio = max(yolo_data.bbox_w / self.image_width, 0.1)
        # Distance estimation: small bbox (~0.1) = 0.4m away, large bbox (~0.4) = 0.2m away
        estimated_distance = 0.5 - (bbox_width_ratio * 0.75)  # 0.5m to 0.2m range
        estimated_distance = max(0.2, min(0.4, estimated_distance))
        
        # Convert normalized image position to real-world angles
        # Assume camera FOV ~60 degrees horizontal, ~45 degrees vertical
        horizontal_angle = norm_x * math.radians(30)  # ±30 degrees
        vertical_angle = norm_y * math.radians(22.5)  # ±22.5 degrees
        
        # Target position in camera frame
        target_x_cam = estimated_distance * math.sin(horizontal_angle)
        target_y_cam = estimated_distance * math.cos(horizontal_angle)
        target_z_cam = -estimated_distance * math.sin(vertical_angle)
        
        # Transform from camera frame to base frame
        # Camera is approximately 0.15m forward, 0.2m up from base
        camera_offset_x = 0.15
        camera_offset_z = 0.2
        
        target_x = target_y_cam + camera_offset_x  # Forward from base
        target_y = target_x_cam  # Left/right from base
        target_z = target_z_cam + camera_offset_z  # Height above base
        
        # ==================== INVERSE KINEMATICS ====================
        # joint1: Base rotation to point at target
        joint1 = math.atan2(target_y, target_x)
        joint1 = max(-math.pi, min(math.pi, joint1))  # Clamp to joint limits
        
        # Calculate reach distance in XY plane and height
        r_xy = math.sqrt(target_x**2 + target_y**2)
        r_z = target_z - self.L1  # Subtract base height
        
        # Distance from shoulder joint to target
        reach_distance = math.sqrt(r_xy**2 + r_z**2)
        
        # Check if target is reachable
        max_reach = self.L2 + self.L3 + self.L4
        if reach_distance > max_reach:
            self.get_logger().warn(
                f"Target at {reach_distance:.3f}m exceeds max reach {max_reach:.3f}m. "
                "Using maximum extension."
            )
            reach_distance = max_reach * 0.95  # Use 95% of max reach
        
        # 2-DOF IK for shoulder and elbow (joints 2 and 3)
        # Treat L3+L4 as single effective link for simplification
        L_eff = self.L3 + self.L4
        
        # Law of cosines for elbow angle (joint3)
        # cos(joint3) = (reach_distance^2 - L2^2 - L_eff^2) / (2 * L2 * L_eff)
        cos_joint3 = (reach_distance**2 - self.L2**2 - L_eff**2) / (2 * self.L2 * L_eff)
        cos_joint3 = max(-1.0, min(1.0, cos_joint3))  # Clamp to valid range
        joint3 = math.acos(cos_joint3)
        
        # Shoulder angle (joint2) 
        # Two components: angle to target + compensation for elbow bend
        angle_to_target = math.atan2(r_z, r_xy)
        # Law of sines to find angle offset
        sin_offset = (L_eff * math.sin(joint3)) / reach_distance
        sin_offset = max(-1.0, min(1.0, sin_offset))
        angle_offset = math.asin(sin_offset)
        joint2 = angle_to_target - angle_offset
        
        # Joint limits for OpenMANIPULATOR-X
        joint2 = max(-1.5, min(1.5, joint2))
        joint3 = max(-1.5, min(1.5, joint3))
        
        # joint4: Wrist angle to keep end effector level/pointing down
        # Compensate for joint2 + joint3 to maintain desired end effector orientation
        # For grabbing from above, we want gripper pointing down (pitch = -pi/2)
        desired_ee_pitch = -math.pi / 3  # -60 degrees (pointing somewhat down)
        joint4 = desired_ee_pitch - (joint2 + joint3)
        joint4 = max(-1.5, min(1.5, joint4))
        
        self.get_logger().info(
            f"IK Solution: target=({target_x:.3f}, {target_y:.3f}, {target_z:.3f}), "
            f"reach={reach_distance:.3f}m, "
            f"joints=[{math.degrees(joint1):.1f}°, {math.degrees(joint2):.1f}°, "
            f"{math.degrees(joint3):.1f}°, {math.degrees(joint4):.1f}°]"
        )
        
        return {
            'joint1': joint1,
            'joint2': joint2,
            'joint3': joint3,
            'joint4': joint4
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
                
                # Confirm we're using the reference frame
                if self.manipulator_reference_frame:
                    self.using_reference_frame = True
                    self.get_logger().info(
                        f"Using manipulator reference frame from class {self.manipulator_reference_frame.clz}"
                    )
                
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
                self.manipulator_reference_frame = None  # Clear reference frame on error
                self.using_reference_frame = False
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
                
                # Approach for grasp - small forward movement
                self.get_logger().info("Approaching for grasp...")
                grasp_positions = {
                    'joint1': arm_positions['joint1'],
                    'joint2': arm_positions['joint2'],  # Keep same angle
                    'joint3': min(1.5, arm_positions['joint3'] + 0.1),  # Extend slightly more
                    'joint4': arm_positions['joint4']   # Keep same wrist angle
                }
                self.teleop_pub.send_arm_trajectory(grasp_positions)
                time.sleep(2.0)
                
                # Close gripper
                self.get_logger().info("Closing gripper...")
                self.teleop_pub.gripper_close()
                time.sleep(2.0)
                
                # Lift object - pull back up from camera level
                self.get_logger().info("Lifting object...")
                lift_positions = {
                    'joint1': arm_positions['joint1'],
                    'joint2': -0.5,   # Lift up from deep camera level position
                    'joint3': 0.4,    # Retract elbow
                    'joint4': 0.6     # Adjust wrist
                }
                self.teleop_pub.send_arm_trajectory(lift_positions)
                time.sleep(2.0)
                
                self.get_logger().info("Grab successful! Holding object...")
                self.set_state(ArmMissionState.HOLDING)
            except Exception as e:  # noqa: B902
                self.get_logger().error(f"Grab failed: {e}")
                self.object_detected = False
                self.manipulator_reference_frame = None  # Clear reference frame on error
                self.using_reference_frame = False
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
                
                # Lower arm slightly to place object
                self.get_logger().info("Lowering arm...")
                self.teleop_pub.send_arm_trajectory({
                    'joint1': 0.0,
                    'joint2': -0.8,   # Lower back down but not as far as camera level
                    'joint3': 0.6,
                    'joint4': 0.8
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
            self.manipulator_reference_frame = None  # Reset reference frame for next object
            self.using_reference_frame = False
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
