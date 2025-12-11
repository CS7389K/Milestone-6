#!/usr/bin/env python3
"""
Part 4: LLM-Guided Object Search and Retrieval

This node extends Part 3 by using an LLM for high-level planning instead of 
direct voice commands. The robot receives natural language instructions that
are translated to atomic actions by a LLaMA model running on a remote PC.

The environment contains three objects: bottle, bear doll, and computer mouse.

Atomic Actions (from LLM):
- TURN_LEFT: Rotate counter-clockwise
- TURN_RIGHT: Rotate clockwise
- MOVE_FORWARD: Move forward
- SCAN: Perform 360° scan
- SEARCH <object>: Begin visual search for specific object
- GRAB: Execute grab sequence
- TRANSPORT_TO <object>: Navigate toward target object
- PLACE: Place held object
- DONE: Mission complete

Workflow:
1. LLM_GUIDED: Subscribe to /llm_action, execute atomic actions
2. When target object detected -> visual servoing (Part 2)
3. CENTERING -> APPROACHING -> GRABBING -> TRANSPORTING -> PLACING -> DONE

Usage:
    ros2 run milestone6 part4
"""

import math
from enum import Enum

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Base and Arm control
from milestone6.teleop.publisher import TeleopPublisher
from milestone6.teleop.subscriber import TeleopSubscriber

# YOLO detection
from milestone6.yolo.subscriber import YOLOSubscriber
from milestone6.yolo.yolo_data import YOLOData


class State(Enum):
    """States for LLM-guided search and retrieval sequence."""
    LLM_GUIDED = 0       # Execute LLM atomic actions
    EXECUTING_CMD = 1    # Executing an atomic action
    CENTERING = 2        # Rotating to center object (Part 2)
    APPROACHING = 3      # Moving forward to grab distance (Part 2)
    GRABBING = 4         # Executing grab sequence (Part 2)
    TRANSPORTING = 5     # Moving with object (LLM-guided)
    SEARCHING_TARGET = 6  # Searching for placement target
    PLACING = 7          # Placing object sequence
    DONE = 8             # Complete


class Part4(Node):
    """
    LLM-guided high-level planning with Part 2 visual servoing integration.
    """

    def __init__(self):
        super().__init__('part4')

        # ------------------- Parameters -------------------
        # LLM action execution parameters
        self.declare_parameter('turn_duration', 1.0)        # seconds
        self.declare_parameter('forward_duration', 0.5)     # seconds
        self.declare_parameter('scan_speed', 0.5)           # rad/s
        self.declare_parameter('forward_speed_llm', 0.2)    # m/s

        # Visual servoing parameters (Part 2)
        # bottle, bear, mouse
        self.declare_parameter('tracking_classes', '39,73,64')
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('bbox_tolerance', 20)
        self.declare_parameter('center_tolerance', 30)
        self.declare_parameter('target_bbox_width', 365)
        self.declare_parameter('forward_speed', 0.05)
        self.declare_parameter('turn_speed', 0.25)
        self.declare_parameter('detection_timeout', 1)
        self.declare_parameter('transport_distance', 0.10)

        # Arm parameters
        self.declare_parameter('grab_joint2', 0.95)
        self.declare_parameter('grab_joint3', -0.65)
        self.declare_parameter('grab_joint4', 0.0)
        self.declare_parameter('grab_vertical_adjust', 0.2)
        self.declare_parameter('grasp_extension', 0.2)
        self.declare_parameter('lift_joint2', -0.5)
        self.declare_parameter('lift_joint3', 0.4)
        self.declare_parameter('lift_joint4', 0.6)
        self.declare_parameter('lower_joint1', 0.0)
        self.declare_parameter('lower_joint2', 0.6)
        self.declare_parameter('lower_joint3', -0.4)
        self.declare_parameter('lower_joint4', 0.6)
        self.declare_parameter('home_joint1', 0.0)
        self.declare_parameter('home_joint2', -1.05)
        self.declare_parameter('home_joint3', 0.35)
        self.declare_parameter('home_joint4', 0.70)
        self.declare_parameter('gripper_open', 0.025)
        self.declare_parameter('gripper_close', -0.015)

        # Get parameters
        self.turn_duration = self.get_parameter('turn_duration').value
        self.forward_duration = self.get_parameter('forward_duration').value
        self.scan_speed = self.get_parameter('scan_speed').value
        self.forward_speed_llm = self.get_parameter('forward_speed_llm').value

        tracking_classes = self.get_parameter('tracking_classes').value
        self.tracking_class_ids = [int(c.strip())
                                   for c in tracking_classes.split(',')]

        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.bbox_tolerance = self.get_parameter('bbox_tolerance').value
        self.center_tolerance = self.get_parameter('center_tolerance').value
        self.target_bbox_width = self.get_parameter('target_bbox_width').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.transport_distance = self.get_parameter(
            'transport_distance').value

        # Arm positions
        self.grab_joint2 = self.get_parameter('grab_joint2').value
        self.grab_joint3 = self.get_parameter('grab_joint3').value
        self.grab_joint4 = self.get_parameter('grab_joint4').value
        self.grab_vertical_adjust = self.get_parameter(
            'grab_vertical_adjust').value
        self.grasp_extension = self.get_parameter('grasp_extension').value
        self.lift_joint2 = self.get_parameter('lift_joint2').value
        self.lift_joint3 = self.get_parameter('lift_joint3').value
        self.lift_joint4 = self.get_parameter('lift_joint4').value
        self.lower_joint1 = self.get_parameter('lower_joint1').value
        self.lower_joint2 = self.get_parameter('lower_joint2').value
        self.lower_joint3 = self.get_parameter('lower_joint3').value
        self.lower_joint4 = self.get_parameter('lower_joint4').value
        self.home_joint1 = self.get_parameter('home_joint1').value
        self.home_joint2 = self.get_parameter('home_joint2').value
        self.home_joint3 = self.get_parameter('home_joint3').value
        self.home_joint4 = self.get_parameter('home_joint4').value
        self.gripper_open = self.get_parameter('gripper_open').value
        self.gripper_close = self.get_parameter('gripper_close').value

        # ------------------- State -------------------
        self.state = State.LLM_GUIDED
        self.current_action = None
        self.action_start_time = None
        self.scan_complete = False
        self.initial_yaw = None

        # Object tracking
        self.current_target = None  # 'bottle', 'bear', or 'mouse'
        self.held_object = None     # Track if holding bottle
        self.placement_target = None  # Target object for placement

        # Detection tracking
        self.latest_detection = None
        self.detection_time = None

        # ------------------- Publishers/Subscribers -------------------
        # Base control
        self.teleop_pub = TeleopPublisher(self)
        self.teleop_sub = TeleopSubscriber(self)

        # YOLO detection
        self.yolo_sub = YOLOSubscriber(self, self._yolo_callback)

        # LLM action subscriber
        self.llm_action_sub = self.create_subscription(
            String,
            '/llm_action',
            self._llm_action_callback,
            10
        )

        # Text-to-speech publisher
        self.tts_pub = self.create_publisher(String, '/text_to_speech', 10)

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

        # ------------------- Timer -------------------
        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().info("Part 4 initialized - LLM-Guided Mission")
        self.get_logger().info(f"Tracking classes: {self.tracking_class_ids}")
        self.get_logger().info("Waiting for LLM actions on /llm_action...")

    # ------------------------------------------------------------------
    # YOLO Callback
    # ------------------------------------------------------------------
    def _yolo_callback(self, data: YOLOData):
        """Process YOLO detections for tracked objects."""
        if not data.detections:
            return

        # Filter for tracking classes
        for det in data.detections:
            if det.class_id in self.tracking_class_ids:
                self.latest_detection = data
                self.detection_time = self.get_clock().now()
                break

    # ------------------------------------------------------------------
    # LLM Action Callback
    # ------------------------------------------------------------------
    def _llm_action_callback(self, msg: String):
        """Process atomic action from LLM."""
        action = msg.data.strip().upper()

        if not action:
            return

        self.get_logger().info(f"[LLM ACTION] {action}")

        # Parse and store action
        if action == 'DONE':
            self.state = State.DONE
            self._speak("Mission complete")
        elif self.state == State.LLM_GUIDED or self.state == State.TRANSPORTING:
            self.current_action = action
            self.action_start_time = None  # Will be set when execution starts
            self.state = State.EXECUTING_CMD

    # ------------------------------------------------------------------
    # Helper Methods
    # ------------------------------------------------------------------
    def _speak(self, text: str):
        """Publish text to be spoken."""
        self.get_logger().info(f"[TTS] {text}")
        msg = String()
        msg.data = text
        self.tts_publisher.publish(msg)

    def _parse_search_target(self, action: str) -> str:
        """Extract target object from SEARCH command."""
        parts = action.split()
        if len(parts) >= 2:
            target = parts[1].lower()
            if 'bottle' in target:
                return 'bottle'
            elif 'bear' in target or 'doll' in target:
                return 'bear'
            elif 'mouse' in target:
                return 'mouse'
        return None

    def _parse_transport_target(self, action: str) -> str:
        """Extract target object from TRANSPORT_TO command."""
        return self._parse_search_target(action)

    def _get_class_id_for_object(self, obj_name: str) -> int:
        """Get COCO class ID for object name."""
        mapping = {
            'bottle': 39,
            'bear': 73,
            'mouse': 64
        }
        return mapping.get(obj_name, 39)

    def _execute_atomic_action(self):
        """Execute the current atomic action from LLM."""
        if self.current_action is None:
            return

        action = self.current_action

        # Set start time if not set
        if self.action_start_time is None:
            self.action_start_time = self.get_clock().now()
            self.scan_complete = False

        elapsed = (self.get_clock().now() -
                   self.action_start_time).nanoseconds / 1e9

        # Execute action based on type
        if action == 'TURN_LEFT':
            if elapsed < self.turn_duration:
                self.teleop_pub.publish_twist(0.0, self.scan_speed)
            else:
                self.stop_base()
                self._finish_action()

        elif action == 'TURN_RIGHT':
            if elapsed < self.turn_duration:
                self.teleop_pub.publish_twist(0.0, -self.scan_speed)
            else:
                self.stop_base()
                self._finish_action()

        elif action == 'MOVE_FORWARD':
            if elapsed < self.forward_duration:
                self.teleop_pub.publish_twist(self.forward_speed_llm, 0.0)
            else:
                self.stop_base()
                self._finish_action()

        elif action == 'SCAN':
            self._execute_scan()

        elif action.startswith('SEARCH'):
            target = self._parse_search_target(action)
            if target:
                self.current_target = target
                self.tracking_class_ids = [
                    self._get_class_id_for_object(target)]
                self._speak(f"Searching for {target}")
                self.get_logger().info(
                    f"Searching for {target} (class {self.tracking_class_ids[0]})")
            self._finish_action()

        elif action == 'GRAB':
            # Transition to visual servoing for grabbing
            if self.detection_is_fresh():
                self.state = State.CENTERING
                self._speak("Engaging visual servoing for grab")
            else:
                self._speak("No object detected, cannot grab")
                self._finish_action()

        elif action.startswith('TRANSPORT_TO'):
            target = self._parse_transport_target(action)
            if target:
                self.placement_target = target
                self.held_object = 'bottle'  # Assume we picked up bottle
                self._speak(f"Transporting to {target}")
                self.state = State.TRANSPORTING
            self._finish_action()

        elif action == 'PLACE':
            # Check if we have a target object in view
            if self.detection_is_fresh() and self.held_object:
                self.state = State.PLACING
                self._speak("Placing object")
            else:
                self._speak("Need to locate placement target first")
                self._finish_action()

        else:
            self.get_logger().warn(f"Unknown action: {action}")
            self._finish_action()

    def _execute_scan(self):
        """Execute 360° scan."""
        if self.initial_yaw is None:
            # Initialize scan
            self.initial_yaw = self.teleop_sub.yaw
            self.scan_complete = False
            self.get_logger().info(
                f"Starting scan from yaw: {self.initial_yaw:.2f}")

        current_yaw = self.teleop_sub.yaw

        # Calculate how far we've rotated
        yaw_diff = abs(current_yaw - self.initial_yaw)
        if yaw_diff > math.pi:
            yaw_diff = 2 * math.pi - yaw_diff

        if yaw_diff >= 2 * math.pi - 0.2:  # Nearly complete rotation
            self.stop_base()
            self.scan_complete = True
            self.initial_yaw = None
            self._speak("Scan complete")
            self._finish_action()
        else:
            # Continue rotating
            self.teleop_pub.publish_twist(0.0, self.scan_speed)

    def _finish_action(self):
        """Complete current atomic action and return to LLM_GUIDED state."""
        self.current_action = None
        self.action_start_time = None

        if self.state == State.EXECUTING_CMD:
            self.state = State.LLM_GUIDED
        elif self.state == State.TRANSPORTING:
            # Stay in transporting state to receive more guidance
            pass

    # ------------------------------------------------------------------
    # Part 2 Helper Methods (Visual Servoing)
    # ------------------------------------------------------------------
    def detection_is_fresh(self):
        """Check if we have a recent detection."""
        if self.last_detection_time is None:
            self.get_logger().debug("No detection time set")
            return False
        elapsed = (self.get_clock().now() -
                   self.last_detection_time).nanoseconds / 1e9
        is_fresh = elapsed < self.detection_timeout
        self.get_logger().debug(
            f"Detection age: {elapsed:.3f}s, fresh: {is_fresh}")
        return is_fresh

    def stop_base(self):
        """Stop base movement."""
        self.teleop_pub.set_velocity(linear_x=0.0, angular_z=0.0)

    def is_centered(self, yolo_data: YOLOData):
        """Check if object is centered in frame."""
        obj_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)
        image_center_x = self.image_width / 2.0
        offset_x = abs(obj_center_x - image_center_x)
        return offset_x <= self.center_tolerance

    def is_at_grab_distance(self, yolo_data: YOLOData):
        """Check if object is at ideal grabbing distance."""
        bbox_error = abs(yolo_data.bbox_w - self.target_bbox_width)
        return bbox_error <= self.bbox_tolerance

    def center_on_object(self, yolo_data: YOLOData):
        """
        Calculate and apply turning velocity to center object.
        Returns True if centered, False otherwise.
        """
        obj_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)
        image_center_x = self.image_width / 2.0
        offset_x = obj_center_x - image_center_x

        if abs(offset_x) <= self.center_tolerance:
            self.stop_base()
            return True

        # Calculate turning velocity (negative offset = object left = turn left)
        angular_ratio = -offset_x / (self.image_width / 2.0)
        angular_ratio = max(-1.0, min(1.0, angular_ratio))  # Clamp to [-1, 1]
        angular_vel = angular_ratio * self.turn_speed

        self.teleop_pub.set_velocity(linear_x=0.0, angular_z=angular_vel)
        self.get_logger().debug(
            f"Centering: offset={offset_x:.1f}px, angular={angular_vel:.2f} rad/s")
        return False

    def approach_object(self, yolo_data: YOLOData):
        """
        Move forward/backward to achieve ideal grabbing distance.
        Returns True if at correct distance, False otherwise.
        """
        bbox_error = yolo_data.bbox_w - self.target_bbox_width

        if abs(bbox_error) <= self.bbox_tolerance:
            self.stop_base()
            return True

        if bbox_error < -self.bbox_tolerance:
            # Too far, move forward
            linear_vel = self.forward_speed
            self.get_logger().debug(
                f"Too far (bbox={yolo_data.bbox_w:.0f}px), moving forward")
        else:
            # Too close, move backward
            linear_vel = -self.forward_speed * 0.5
            self.get_logger().debug(
                f"Too close (bbox={yolo_data.bbox_w:.0f}px), backing up")

        self.teleop_pub.set_velocity(linear_x=linear_vel, angular_z=0.0)
        return False

    def send_arm_trajectory(self, positions: dict, time_sec: float = 2.0):
        """
        Send arm trajectory using FollowJointTrajectory action.

        Args:
            positions: Dict mapping joint names to target positions (radians)
            time_sec: Time to reach target position

        Returns:
            Future for the action goal
        """
        if not self.arm_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Arm controller not available!")
            return None

        # Create trajectory
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [positions.get(j, 0.0) for j in joint_names]
        point.time_from_start = Duration(
            sec=int(time_sec),
            nanosec=int((time_sec % 1.0) * 1e9)
        )
        trajectory.points = [point]

        # Send goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        future = self.arm_client.send_goal_async(goal)
        self.get_logger().info(f"Sent arm trajectory: {positions}")
        return future

    def send_gripper_command(self, position: float):
        """
        Send gripper command using GripperCommand action.

        Args:
            position: Gripper position (0.025 = open, -0.015 = close)

        Returns:
            Future for the action goal
        """
        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Gripper controller not available!")
            return None

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = -1.0

        future = self.gripper_client.send_goal_async(goal)
        action = "Opening" if position > 0 else "Closing"
        self.get_logger().info(f"{action} gripper (pos={position})")
        return future

    def calculate_grab_position(self, yolo_data: YOLOData):
        """
        Calculate arm position for grabbing based on object location.
        Uses simple visual servoing approach.
        """
        obj_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)
        obj_center_y = yolo_data.bbox_y + (yolo_data.bbox_h / 2.0)

        # Normalize positions
        norm_x = (obj_center_x - (self.image_width / 2.0)) / \
            (self.image_width / 2.0)
        norm_y = (obj_center_y - (self.image_height / 2.0)) / \
            (self.image_height / 2.0)

        # Joint 1: Pan (base rotation) - camera FOV ~62°
        camera_hfov_rad = math.radians(62.0)
        joint1 = norm_x * (camera_hfov_rad / 2.0)
        joint1 = max(-1.5, min(1.5, joint1))  # Safety limits

        # Joints 2-4: Forward reach configuration
        # These values position arm for grabbing at optimal distance
        joint2 = self.grab_joint2
        joint3 = self.grab_joint3
        joint4 = self.grab_joint4

        # Adjust reach based on vertical position
        if norm_y > 0.2:  # Object low in frame
            joint2 += self.grab_vertical_adjust  # Reach down more

        return {
            'joint1': joint1,
            'joint2': joint2,
            'joint3': joint3,
            'joint4': joint4
        }

    # ------------------------------------------------------------------
    # State Machine
    # ------------------------------------------------------------------
    def tick(self):
        """Main state machine tick."""
        if self.state == State.LLM_GUIDED:
            # Waiting for LLM action
            pass

        elif self.state == State.EXECUTING_CMD:
            # Execute atomic action
            self._execute_atomic_action()

        elif self.state == State.CENTERING:
            # Center on detected object (Part 2)
            if self.detection_is_fresh():
                if self.is_centered(self.latest_detection):
                    self.stop_base()
                    self.state = State.APPROACHING
                    self.get_logger().info("Centered! Moving to approach...")
                else:
                    self.center_on_object(self.latest_detection)
            else:
                self.stop_base()
                self.get_logger().warn("Lost object during centering")
                self.state = State.LLM_GUIDED

        elif self.state == State.APPROACHING:
            # Approach to grab distance (Part 2)
            if self.detection_is_fresh():
                if self.is_at_grab_distance(self.latest_detection):
                    self.stop_base()
                    self.state = State.GRABBING
                    self.get_logger().info("At grab distance! Executing grab...")
                    self._execute_grab_sequence()
                else:
                    self.approach_object(self.latest_detection)
            else:
                self.stop_base()
                self.get_logger().warn("Lost object during approach")
                self.state = State.LLM_GUIDED

        elif self.state == State.GRABBING:
            # Grab sequence executing (handled by timer in sequence)
            pass

        elif self.state == State.TRANSPORTING:
            # Continue accepting LLM actions while transporting
            pass

        elif self.state == State.PLACING:
            # Execute placement sequence
            self._execute_place_sequence()

        elif self.state == State.DONE:
            self.stop_base()
            # Mission complete

    # ------------------------------------------------------------------
    # Action Sequences
    # ------------------------------------------------------------------
    def _execute_grab_sequence(self):
        """Multi-step grabbing sequence using FollowJointTrajectory."""
        # Step 0: Open gripper
        if self.grab_step == 0:
            self.get_logger().info("[GRAB] Step 0: Opening gripper...")
            self.gripper_action_future = self.send_gripper_command(
                self.gripper_open)
            self.step_start_time = self.get_clock().now()
            self.grab_step = 1

        # Step 1: Wait for gripper to open
        elif self.grab_step == 1:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.get_logger().info(
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
                self.get_logger().info("[GRAB] Step 2: Extending to grasp...")
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
                self.get_logger().info("[GRAB] Step 3: Closing gripper...")
                self.gripper_action_future = self.send_gripper_command(
                    self.gripper_close)
                self.step_start_time = self.get_clock().now()
                self.grab_step = 4

        # Step 4: Wait for gripper, then lift
        elif self.grab_step == 4:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.get_logger().info("[GRAB] Step 4: Lifting object...")
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
                self.get_logger().info(
                    "[GRAB] Object grabbed! Moving to TRANSPORTING...")
                self.grab_step = 0
                self.state = State.TRANSPORTING

    def _execute_release_sequence(self):
        """Multi-step release sequence."""

        # Step 0: Lower arm
        if self.release_step == 0:
            self.get_logger().info(
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
                self.get_logger().info(
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
                self.get_logger().info(
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
                self.get_logger().info("[RELEASE] Release complete!")
                self.release_step = 0
                self.state = State.DONE

    def shutdown(self):
        """Cleanup on shutdown."""
        self.get_logger().info("Shutting down Part 2...")
        self.stop_base()
        self.timer.cancel()
        self.teleop_pub.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Part4()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down Part 4...")
    except Exception as e:
        print(f"Error in Part 4: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
