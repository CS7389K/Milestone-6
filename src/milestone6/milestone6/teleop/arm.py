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
        
        # ------------------- Joint State Tracking -------------------
        self.get_logger().info("Starting Joint State Subscriber...")
        self.teleop_sub = TeleopSubscriber(self)
        
        # Track joint state warnings
        self.joint_state_warning_count = 0
        self.joint_state_warning_interval = 50  # Warn every 5 seconds (50 * 0.1s)

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
        
        # Sub-state tracking for multi-step operations
        self.preparing_step = 0
        self.grabbing_step = 0
        self.releasing_step = 0
        self.step_start_time = None
        self.motion_timeout = 10.0  # seconds to wait for motion completion (increased from 5.0)
        self.max_retries = 3
        self.retry_count = 0
        
        # Target position tracking for motion completion verification
        self.preparing_target = {}
        self.grabbing_target = {}
        self.releasing_target = {}

        # Timing
        self.hold_start_time = None
        self.last_attempt_time = None
        self.attempt_cooldown = 10.0  # Wait 10 seconds between grab attempts

        class_name = COCO_CLASSES.get(self.target_class, 'unknown')
        self.get_logger().info(f"Tracking COCO class: '{class_name}' (ID: {self.target_class})")
        self.get_logger().info("TeleopArmV1 ready. Waiting for object detection...")
    
    
    def wait_for_motion_complete(self, target_positions: dict, tolerance: float = 0.15) -> bool:
        """
        Check if arm has reached target positions within tolerance.
        
        Args:
            target_positions: Dictionary of target joint positions
            tolerance: Acceptable position error in radians
            
        Returns:
            True if all joints are within tolerance
        """
        if not self.teleop_sub.have_joint_states:
            self.get_logger().warn("[MOTION_CHECK] No joint states available yet")
            return False
        
        current = self.teleop_sub.joint_positions
        max_error = 0.0
        all_within_tolerance = True
        errors_detail = []
        
        for joint_name, target_pos in target_positions.items():
            if joint_name not in current:
                continue
            error = abs(current[joint_name] - target_pos)
            errors_detail.append(f"{joint_name}: {error:.3f}rad")
            max_error = max(max_error, error)
            if error > tolerance:
                all_within_tolerance = False
        
        if all_within_tolerance:
            self.get_logger().info(f"✓ Motion complete! Max error: {max_error:.3f} rad [{', '.join(errors_detail)}]")
        else:
            self.get_logger().info(f"⟳ Moving... Max error: {max_error:.3f} rad (tol: {tolerance:.3f}) [{', '.join(errors_detail)}]")
        
        return all_within_tolerance


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
        Calculate arm joint positions to reach toward detected object using simple visual servoing.
        
        Instead of trying to calculate exact IK, we use validated poses and adjust incrementally:
        1. Start from a known "reach forward" pose
        2. Adjust joint1 (base rotation) to align with object horizontally
        3. Use bbox size to estimate if we need to reach further or closer
        
        This is more robust than IK because we use proven poses as starting points.
        """
        # Check if we have current joint positions
        if not self.teleop_sub.have_joint_states:
            self.get_logger().error("No joint states available! Cannot calculate safe trajectory.")
            # Return safe reaching pose
            return {
                'joint1': 0.0,
                'joint2': 0.5,   # Forward reach (positive = down/forward)
                'joint3': -0.3,  # Extend elbow (negative = extend)
                'joint4': 0.0    # Level gripper
            }
        
        # Get current joint positions
        current_pos = self.teleop_sub.joint_positions
        
        # Calculate object center in image coordinates
        obj_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)
        obj_center_y = yolo_data.bbox_y + (yolo_data.bbox_h / 2.0)
        
        # Normalize horizontal position to [-1, 1]
        norm_x = (obj_center_x - (self.image_width / 2.0)) / (self.image_width / 2.0)
        norm_y = (obj_center_y - (self.image_height / 2.0)) / (self.image_height / 2.0)
        
        # --- JOINT 1: Pan to align with object ---
        # Camera FOV ~62°, map to joint1 rotation
        camera_hfov_rad = math.radians(62.0)
        desired_joint1 = norm_x * (camera_hfov_rad / 2.0)
        joint1 = max(-1.5, min(1.5, desired_joint1))  # Limit range
        
        # --- DISTANCE ESTIMATION from bbox width ---
        # Larger bbox = closer object, need less reach
        bbox_width_normalized = yolo_data.bbox_w / self.image_width
        
        # FORWARD REACHING POSES (positive joint2 = forward/down)
        if bbox_width_normalized > 0.35:  # Very close (<20cm)
            joint2 = 0.3    # Slight forward
            joint3 = -0.2   # Slight extension
            joint4 = 0.0    # Level
            self.get_logger().info(f"Object VERY CLOSE (bbox={yolo_data.bbox_w:.0f}px), short reach")
        elif bbox_width_normalized > 0.25:  # Close (~25cm)
            joint2 = 0.5    # Moderate forward
            joint3 = -0.3   # Moderate extension
            joint4 = 0.0    # Level
            self.get_logger().info(f"Object CLOSE (bbox={yolo_data.bbox_w:.0f}px), medium reach")
        else:  # Far (>30cm)
            joint2 = 0.7    # Full forward
            joint3 = -0.4   # Full extension
            joint4 = 0.0    # Level
            self.get_logger().info(f"Object FAR (bbox={yolo_data.bbox_w:.0f}px), full reach")
        
        # Adjust vertical based on object position in frame
        # If object is low in frame (high norm_y), reach down more
        if norm_y > 0.3:  # Object in lower half of frame
            joint2 += 0.2  # Reach down more
            self.get_logger().info(f"Object LOW in frame (y={obj_center_y:.0f}), reaching down")
        
        self.get_logger().info(
            f"Calculated pose: j1={joint1:.2f} (pan), j2={joint2:.2f} (reach), "
            f"j3={joint3:.2f} (extend), j4={joint4:.2f} (level) | "
            f"Object at ({obj_center_x:.0f}, {obj_center_y:.0f})px, bbox_w={yolo_data.bbox_w:.0f}px"
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
                # Check cooldown period - don't retry immediately after failed attempt
                if self.last_attempt_time is not None:
                    cooldown_elapsed = (self.get_clock().now() - self.last_attempt_time).nanoseconds / 1e9
                    if cooldown_elapsed < self.attempt_cooldown:
                        remaining = self.attempt_cooldown - cooldown_elapsed
                        if int(remaining) != int(remaining + 0.1):  # Log once per second
                            self.get_logger().info(f"⏳ Cooldown active: waiting {remaining:.0f}s before next attempt...")
                        return
                
                # Check if detection is recent (within timeout)
                if self.last_detection_time:
                    elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
                    if elapsed < self.detection_timeout:
                        self.get_logger().info(f"\n{'='*60}\n🎯 Object detected (class {self.last_yolo_data.clz})! Starting grab sequence...\n{'='*60}")
                        self.teleop_pub.set_velocity(linear_x=0.0, angular_z=0.0)
                        self.retry_count = 0  # Reset retry counter
                        self.last_attempt_time = self.get_clock().now()  # Record attempt time
                        self.set_state(ArmMissionState.PREPARING)
                    else:
                        self.get_logger().debug(f"Detection too old ({elapsed:.1f}s), waiting...")
            return

        # ------------------- STATE: PREPARING -------------------
        elif self.state == ArmMissionState.PREPARING:
            # Wait for joint states before proceeding
            if not self.teleop_sub.have_joint_states:
                self.joint_state_warning_count += 1
                if self.joint_state_warning_count % self.joint_state_warning_interval == 1:
                    self.get_logger().error(
                        "=" * 60 + "\n"
                        "NO JOINT STATES AVAILABLE!\n"
                        "The /joint_states topic is not publishing data.\n"
                        "\n"
                        "SOLUTION: You must run hardware.launch.py FIRST:\n"
                        "  Terminal 1: ros2 launch turtlebot3_manipulation_bringup hardware.launch.py\n"
                        "  Terminal 2: ./run_part2.sh  (this script)\n"
                        "\n"
                        "Or check if controllers are running:\n"
                        "  ros2 topic list | grep joint_states\n"
                        "  ros2 topic echo /joint_states\n"
                        "=" * 60
                    )
                else:
                    self.get_logger().warn("Waiting for joint states...")
                return
            
            # Multi-step state machine for PREPARING
            if self.preparing_step == 0:
                # Step 0: Initialize
                self.get_logger().info("\n[PREPARING] Step 0: Opening gripper and positioning arm...")
                self.teleop_pub.set_velocity(linear_x=0.0, angular_z=0.0)
                
                if self.last_yolo_data is None:
                    self.get_logger().error("No YOLO data available, returning to WAITING")
                    self.set_state(ArmMissionState.WAITING)
                    return
                
                # Log current joint positions
                current = self.teleop_sub.joint_positions
                self.get_logger().info(
                    f"Current arm: j1={current.get('joint1', 0):.2f}, "
                    f"j2={current.get('joint2', 0):.2f}, "
                    f"j3={current.get('joint3', 0):.2f}, "
                    f"j4={current.get('joint4', 0):.2f}"
                )
                
                # Confirm we're using the reference frame
                if self.manipulator_reference_frame:
                    self.using_reference_frame = True
                    self.get_logger().info(
                        f"Using manipulator reference frame from class {self.manipulator_reference_frame.clz}"
                    )
                
                # Open gripper
                self.get_logger().info("[PREPARING] Step 1: Opening gripper...")
                self.teleop_pub.gripper_open()
                self.preparing_step = 1
                self.step_start_time = self.get_clock().now()
                
            elif self.preparing_step == 1:
                # Step 1: Wait for gripper to open (give it 1.5 seconds)
                elapsed = (self.get_clock().now() - self.step_start_time).nanoseconds / 1e9
                if elapsed >= 1.5:
                    # Calculate and move to target position
                    self.get_logger().info("[PREPARING] Step 2: Moving arm to pre-grab position...")
                    target_positions = self.calculate_arm_position_from_object(self.last_yolo_data)
                    self.get_logger().info(f"[PREPARING] Target: j1={target_positions['joint1']:.2f}, j2={target_positions['joint2']:.2f}, j3={target_positions['joint3']:.2f}, j4={target_positions['joint4']:.2f}")
                    self.teleop_pub.send_arm_trajectory(target_positions)
                    self.preparing_target = target_positions
                    self.preparing_step = 2
                    self.step_start_time = self.get_clock().now()
                    
            elif self.preparing_step == 2:
                # Step 2: Wait for arm to reach position
                elapsed = (self.get_clock().now() - self.step_start_time).nanoseconds / 1e9
                
                if self.wait_for_motion_complete(self.preparing_target, tolerance=0.15):
                    self.get_logger().info("[PREPARING] ✓ Arm reached target! Starting grab sequence...")
                    self.preparing_step = 0
                    self.set_state(ArmMissionState.GRABBING)
                elif elapsed > self.motion_timeout:
                    self.retry_count += 1
                    if self.retry_count >= self.max_retries:
                        self.get_logger().error(
                            f"✗ [PREPARING] Motion timeout after {elapsed:.1f}s "
                            f"({self.retry_count}/{self.max_retries} retries). "
                            f"ABORTING - returning to WAITING state."
                        )
                        self.preparing_step = 0
                        self.retry_count = 0
                        self.set_state(ArmMissionState.WAITING)
                    else:
                        self.get_logger().warn(
                            f"⚠ [PREPARING] Motion timeout after {elapsed:.1f}s "
                            f"(retry {self.retry_count}/{self.max_retries}). Proceeding anyway..."
                        )
                        self.preparing_step = 0
                        self.set_state(ArmMissionState.GRABBING)
            return

        # ------------------- STATE: GRABBING -------------------
        elif self.state == ArmMissionState.GRABBING:
            if self.last_yolo_data is None:
                self.get_logger().error("No YOLO data available, returning to WAITING")
                self.object_detected = False
                self.set_state(ArmMissionState.WAITING)
                return
            
            # Multi-step state machine for GRABBING
            if self.grabbing_step == 0:
                # Step 0: Recalculate and position arm towards object
                self.get_logger().info("\n[GRABBING] Step 0: Positioning arm towards object...")
                arm_positions = self.calculate_arm_position_from_object(self.last_yolo_data)
                self.get_logger().info(f"[GRABBING] Target: j1={arm_positions['joint1']:.2f}, j2={arm_positions['joint2']:.2f}, j3={arm_positions['joint3']:.2f}, j4={arm_positions['joint4']:.2f}")
                self.teleop_pub.send_arm_trajectory(arm_positions)
                self.grabbing_target = arm_positions
                self.grabbing_step = 1
                self.step_start_time = self.get_clock().now()
                
            elif self.grabbing_step == 1:
                # Step 1: Wait for arm positioning
                elapsed = (self.get_clock().now() - self.step_start_time).nanoseconds / 1e9
                if self.wait_for_motion_complete(self.grabbing_target, tolerance=0.15):
                    # Approach for grasp - extend slightly more
                    self.get_logger().info("[GRABBING] Step 1: ✓ Positioned! Now approaching for grasp...")
                    grasp_positions = {
                        'joint1': self.grabbing_target['joint1'],
                        'joint2': self.grabbing_target['joint2'],
                        'joint3': min(1.5, self.grabbing_target['joint3'] + 0.15),  # Extend more
                        'joint4': self.grabbing_target['joint4']
                    }
                    self.teleop_pub.send_arm_trajectory(grasp_positions)
                    self.grabbing_target = grasp_positions
                    self.grabbing_step = 2
                    self.step_start_time = self.get_clock().now()
                elif elapsed > self.motion_timeout:
                    self.retry_count += 1
                    if self.retry_count >= self.max_retries:
                        self.get_logger().error(f"✗ [GRABBING] Positioning timeout after {elapsed:.1f}s. ABORTING.")
                        self.grabbing_step = 0
                        self.retry_count = 0
                        self.set_state(ArmMissionState.WAITING)
                    else:
                        self.get_logger().warn(f"⚠ [GRABBING] Positioning timeout (retry {self.retry_count}/{self.max_retries}), proceeding...")
                        self.grabbing_step = 2
                    
            elif self.grabbing_step == 2:
                # Step 2: Wait for approach, then close gripper
                elapsed = (self.get_clock().now() - self.step_start_time).nanoseconds / 1e9
                if elapsed >= 2.5 or self.wait_for_motion_complete(self.grabbing_target, tolerance=0.15):
                    self.get_logger().info("[GRABBING] Step 2: ✓ Approached! Closing gripper...")
                    self.teleop_pub.gripper_close()
                    self.grabbing_step = 3
                    self.step_start_time = self.get_clock().now()
                    
            elif self.grabbing_step == 3:
                # Step 3: Wait for gripper to close, then lift
                elapsed = (self.get_clock().now() - self.step_start_time).nanoseconds / 1e9
                if elapsed >= 2.0:
                    self.get_logger().info("[GRABBING] Step 3: ✓ Gripper closed! Lifting object...")
                    lift_positions = {
                        'joint1': self.grabbing_target['joint1'],
                        'joint2': -0.5,   # Lift up
                        'joint3': 0.4,    # Retract elbow
                        'joint4': 0.6     # Adjust wrist
                    }
                    self.teleop_pub.send_arm_trajectory(lift_positions)
                    self.grabbing_target = lift_positions
                    self.grabbing_step = 4
                    self.step_start_time = self.get_clock().now()
                    
            elif self.grabbing_step == 4:
                # Step 4: Wait for lift to complete
                elapsed = (self.get_clock().now() - self.step_start_time).nanoseconds / 1e9
                if self.wait_for_motion_complete(self.grabbing_target, tolerance=0.15):
                    self.get_logger().info("[GRABBING] Step 4: ✓ Object lifted! Grab successful!\n" + "="*60)
                    self.grabbing_step = 0
                    self.retry_count = 0  # Reset for next operation
                    self.set_state(ArmMissionState.HOLDING)
                elif elapsed > self.motion_timeout:
                    self.retry_count += 1
                    if self.retry_count >= self.max_retries:
                        self.get_logger().error(f"✗ [GRABBING] Lift timeout after {elapsed:.1f}s. ABORTING.")
                        self.grabbing_step = 0
                        self.retry_count = 0
                        self.set_state(ArmMissionState.WAITING)
                    else:
                        self.get_logger().warn(f"⚠ [GRABBING] Lift timeout (retry {self.retry_count}/{self.max_retries}), holding anyway...")
                        self.grabbing_step = 0
                        self.set_state(ArmMissionState.HOLDING)
            return

        # ------------------- STATE: HOLDING -------------------
        elif self.state == ArmMissionState.HOLDING:
            # Start holding timer if not set
            if self.hold_start_time is None:
                self.get_logger().info("\n[HOLDING] Holding object for 2 seconds...")
                self.hold_start_time = self.get_clock().now()
            
            # Hold for 2 seconds, then release
            elapsed = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.hold_start_time = None
                self.get_logger().info("[HOLDING] Time's up! Releasing object...")
                self.set_state(ArmMissionState.RELEASING)
            return

        # ------------------- STATE: RELEASING -------------------
        elif self.state == ArmMissionState.RELEASING:
            # Multi-step state machine for RELEASING
            if self.releasing_step == 0:
                # Step 0: Lower arm to place object
                self.get_logger().info("RELEASING STATE: Lowering arm...")
                lower_positions = {
                    'joint1': 0.0,
                    'joint2': -0.8,   # Lower back down
                    'joint3': 0.6,
                    'joint4': 0.8
                }
                self.teleop_pub.send_arm_trajectory(lower_positions)
                self.releasing_target = lower_positions
                self.releasing_step = 1
                self.step_start_time = self.get_clock().now()
                
            elif self.releasing_step == 1:
                # Step 1: Wait for lowering, then open gripper
                elapsed = (self.get_clock().now() - self.step_start_time).nanoseconds / 1e9
                if self.wait_for_motion_complete(self.releasing_target, tolerance=0.15):
                    self.get_logger().info("Step 2: Opening gripper...")
                    self.teleop_pub.gripper_open()
                    self.releasing_step = 2
                    self.step_start_time = self.get_clock().now()
                elif elapsed > self.motion_timeout:
                    self.get_logger().warn("Lower timeout, proceeding...")
                    self.releasing_step = 2
                    
            elif self.releasing_step == 2:
                # Step 2: Wait for gripper to open, then retract
                elapsed = (self.get_clock().now() - self.step_start_time).nanoseconds / 1e9
                if elapsed >= 1.5:
                    self.get_logger().info("Step 3: Retracting arm...")
                    retract_positions = {
                        'joint1': 0.0,
                        'joint2': -0.3,
                        'joint3': 0.1,
                        'joint4': 0.6
                    }
                    self.teleop_pub.send_arm_trajectory(retract_positions)
                    self.releasing_target = retract_positions
                    self.releasing_step = 3
                    self.step_start_time = self.get_clock().now()
                    
            elif self.releasing_step == 3:
                # Step 3: Wait for retraction to complete
                elapsed = (self.get_clock().now() - self.step_start_time).nanoseconds / 1e9
                if self.wait_for_motion_complete(self.releasing_target, tolerance=0.15):
                    self.get_logger().info("Release successful!")
                    self.releasing_step = 0
                    self.set_state(ArmMissionState.DONE)
                elif elapsed > self.motion_timeout:
                    self.get_logger().warn("Retract timeout, proceeding to DONE")
                    self.releasing_step = 0
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
