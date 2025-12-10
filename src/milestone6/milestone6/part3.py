
#!/usr/bin/env python3
"""
Part 3: Voice-Guided Object Search and Retrieval

This node extends Part 2 by adding voice-controlled initial search phase.
The robot responds to voice commands to help locate the bottle:
- "turn right" / "rotate right" - Rotate clockwise
- "turn left" / "rotate left" - Rotate counter-clockwise  
- "move forward" / "go forward" - Move forward
- "scan" / "look around" - Perform 360° scan

Workflow:
1. VOICE_SEARCH: Robot says "Ready for Command", listens for voice commands
2. When bottle detected -> transition to Part 2's visual servoing
3. CENTERING -> APPROACHING -> GRABBING -> TRANSPORTING -> RELEASING -> DONE

Usage:
    ros2 run milestone6 part3
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

# Import backends for voice interaction
from milestone6.milestone6.nlp.espeak import EspeakBackend

# Base and Arm control
from milestone6.teleop.publisher import TeleopPublisher
from milestone6.teleop.subscriber import TeleopSubscriber
from milestone6.util.coco import COCO_CLASSES

# YOLO detection
from milestone6.yolo.subscriber import YOLOSubscriber
from milestone6.yolo.yolo_data import YOLOData


class State(Enum):
    """States for voice-guided search and grab sequence."""
    VOICE_SEARCH = 0   # Voice-controlled search phase
    EXECUTING_CMD = 1  # Executing a voice command
    CENTERING = 2      # Rotating to center object (Part 2)
    APPROACHING = 3    # Moving forward to grab distance (Part 2)
    GRABBING = 4       # Executing grab sequence (Part 2)
    TRANSPORTING = 5   # Moving forward 1 meter (Part 2)
    RELEASING = 6      # Placing and retracting (Part 2)
    DONE = 7           # Complete


class Part3(Node):
    """
    Voice-guided search with Part 2 visual servoing integration.
    """

    def __init__(self):
        super().__init__('part3')

        # ------------------- Parameters -------------------
        # Voice search parameters
        self.declare_parameter('turn_duration', 1.0)        # seconds
        self.declare_parameter('forward_duration', 0.5)     # seconds
        self.declare_parameter('scan_speed', 0.5)           # rad/s
        # rad/s for voice cmds
        self.declare_parameter('turn_speed_voice', 1.0)
        self.declare_parameter('forward_speed_voice', 0.2)  # m/s
        self.declare_parameter('audio_file', '/tmp/voice_input.wav')
        # seconds between prompts
        self.declare_parameter('prompt_delay', 1.0)
        # Audio recording parameters
        self.declare_parameter('audio_duration', 5)        # seconds
        self.declare_parameter('audio_sample_rate', 16000)  # Hz
        # Amplitude threshold for voice detection
        self.declare_parameter('audio_threshold', 500)
        # Whisper subscriber usage (true for distributed, false for local)
        self.declare_parameter('use_whisper', True)

        # Part 2 parameters (visual servoing)
        self.declare_parameter('tracking_classes', '39')
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('bbox_tolerance', 20)
        self.declare_parameter('center_tolerance', 30)
        self.declare_parameter('target_bbox_width', 180)
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_speed', 1.0)
        self.declare_parameter('detection_timeout', 1)
        self.declare_parameter('transport_distance', 1.0)

        # Arm parameters
        self.declare_parameter('grab_joint2', 0.5)
        self.declare_parameter('grab_joint3', -0.3)
        self.declare_parameter('grab_joint4', 0.0)
        self.declare_parameter('grab_vertical_adjust', 0.2)
        self.declare_parameter('grasp_extension', 0.2)
        self.declare_parameter('lift_joint2', -0.5)
        self.declare_parameter('lift_joint3', 0.4)
        self.declare_parameter('lift_joint4', 0.6)
        self.declare_parameter('lower_joint1', 0.0)
        self.declare_parameter('lower_joint2', 0.3)
        self.declare_parameter('lower_joint3', -0.2)
        self.declare_parameter('lower_joint4', 0.0)
        self.declare_parameter('home_joint1', 0.0)
        self.declare_parameter('home_joint2', -1.05)
        self.declare_parameter('home_joint3', 0.35)
        self.declare_parameter('home_joint4', 0.70)
        self.declare_parameter('gripper_open', 0.025)
        self.declare_parameter('gripper_close', -0.025)

        # Get parameters
        self.turn_duration = self.get_parameter('turn_duration').value
        self.forward_duration = self.get_parameter('forward_duration').value
        self.scan_speed = self.get_parameter('scan_speed').value
        self.turn_speed_voice = self.get_parameter('turn_speed_voice').value
        self.forward_speed_voice = self.get_parameter(
            'forward_speed_voice').value
        self.audio_file = self.get_parameter('audio_file').value
        self.prompt_delay = self.get_parameter('prompt_delay').value
        self.audio_duration = self.get_parameter('audio_duration').value
        self.audio_sample_rate = self.get_parameter('audio_sample_rate').value
        self.audio_threshold = self.get_parameter('audio_threshold').value
        self.use_whisper = self.get_parameter('use_whisper').value

        tracking_classes = self.get_parameter('tracking_classes').value
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

        # Parse tracking classes
        if isinstance(tracking_classes, str):
            self.tracking_classes = [
                int(c.strip()) for c in tracking_classes.split(',') if c.strip()]
        else:
            self.tracking_classes = [int(tracking_classes)]

        # ------------------- Voice Backends -------------------
        self.get_logger().info("Initializing voice backends...")
        self.espeak = EspeakBackend()

        # Subscribe to audio commands from Whisper publisher
        if self.use_whisper:
            self.get_logger().info("Subscribing to /voice_transcription topic...")
            self.audio_command_sub = self.create_subscription(
                String,
                '/voice_transcription',
                self._audio_command_callback,
                10
            )
            self.last_audio_command = None
        else:
            self.get_logger().warn("Whisper disabled - voice commands will not work!")
            self.audio_command_sub = None

        # ------------------- YOLO Subscriber -------------------
        self.get_logger().info("Starting YOLO Subscriber...")
        self.yolo_subscriber = YOLOSubscriber(self, self._yolo_callback)

        # ------------------- Teleop Control -------------------
        self.get_logger().info("Starting Teleop Publisher...")
        self.teleop_pub = TeleopPublisher(self)

        self.get_logger().info("Starting Joint State Subscriber...")
        self.teleop_sub = TeleopSubscriber(self)

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

        # ------------------- State Machine -------------------
        self.state = State.VOICE_SEARCH
        self.last_detection_time = None
        self.last_yolo_data = None

        # Voice command execution
        self.cmd_start_time = None
        self.cmd_duration = 0.0
        self.scanning = False
        self.scan_start_angle = None

        # Part 2 state tracking
        self.grab_step = 0
        self.release_step = 0
        self.step_start_time = None
        self.transport_start_time = None
        self.transport_duration = None
        self.arm_action_future = None
        self.gripper_action_future = None

        # Voice prompt timing
        self.last_prompt_time = None

        # ------------------- Timer -------------------
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz

        class_names = [COCO_CLASSES.get(cls, f'unknown({cls})')
                       for cls in self.tracking_classes]
        self.get_logger().info(
            f"Tracking COCO classes: {', '.join([f'{name} (ID: {cls})' for name, cls in zip(class_names, self.tracking_classes)])}")
        self.get_logger().info("Part 3 Node Initialized.")
        self.get_logger().info("Starting voice-guided search phase...")

        # Give initial prompt
        self._speak("Ready for Command")

    # ------------------------------------------------------------------
    # YOLO Callback
    # ------------------------------------------------------------------
    def _yolo_callback(self, data: YOLOData):
        """Process YOLO detections."""
        if data.clz not in self.tracking_classes:
            return

        self.last_detection_time = self.get_clock().now()
        self.last_yolo_data = data

        # If in voice search and we detect bottle, announce it
        if self.state == State.VOICE_SEARCH:
            class_name = COCO_CLASSES.get(data.clz, f'unknown({data.clz})')
            self.get_logger().info(
                f"[VOICE_SEARCH] {class_name} detected! "
                f"BBox: ({data.bbox_x:.0f}, {data.bbox_y:.0f}, {data.bbox_w:.0f}x{data.bbox_h:.0f})"
            )

    # ------------------------------------------------------------------
    # Voice Interaction Methods
    # ------------------------------------------------------------------
    def _speak(self, text: str):
        """Use espeak to speak text."""
        self.get_logger().info(f"[ESPEAK] {text}")
        try:
            self.espeak(text)
        except Exception as e:
            self.get_logger().error(f"espeak failed: {e}")

    def _audio_command_callback(self, msg: String):
        """Callback for audio commands from Whisper publisher."""
        self.last_audio_command = msg.data
        self.get_logger().info(f"[AUDIO] Received command: '{msg.data}'")

    def _parse_command(self, text: str) -> str:
        """
        Parse voice input to atomic command.
        Returns: 'turn_right', 'turn_left', 'forward', 'scan', or 'unknown'
        """
        text = text.lower().strip()

        # Turn right
        if any(phrase in text for phrase in ['turn right', 'rotate right', 'right']):
            return 'turn_right'

        # Turn left
        if any(phrase in text for phrase in ['turn left', 'rotate left', 'left']):
            return 'turn_left'

        # Move forward
        if any(phrase in text for phrase in ['move forward', 'go forward', 'forward', 'ahead']):
            return 'forward'

        # Scan
        if any(phrase in text for phrase in ['scan', 'look around', 'search']):
            return 'scan'

        return 'unknown'

    def _execute_voice_command(self, cmd: str):
        """Start executing a voice command."""
        if cmd == 'turn_right':
            self._speak("Turning right")
            self.teleop_pub.set_velocity(
                linear_x=0.0, angular_z=-self.turn_speed_voice)
            self.cmd_duration = self.turn_duration

        elif cmd == 'turn_left':
            self._speak("Turning left")
            self.teleop_pub.set_velocity(
                linear_x=0.0, angular_z=self.turn_speed_voice)
            self.cmd_duration = self.turn_duration

        elif cmd == 'forward':
            self._speak("Moving forward")
            self.teleop_pub.set_velocity(
                linear_x=self.forward_speed_voice, angular_z=0.0)
            self.cmd_duration = self.forward_duration

        elif cmd == 'scan':
            pass
            # self._speak("Scanning")
            # self.scanning = True
            # self.scan_start_angle = self.teleop_sub.joint_positions.get(
            #     'joint1', 0.0)
            # self.teleop_pub.set_velocity(
            #     linear_x=0.0, angular_z=self.scan_speed)
            # self.cmd_duration = (2 * math.pi) / self.scan_speed  # 360 degrees

        self.cmd_start_time = self.get_clock().now()
        self.state = State.EXECUTING_CMD

    # ------------------------------------------------------------------
    # Part 2 Helper Methods (Visual Servoing)
    # ------------------------------------------------------------------
    def detection_is_fresh(self):
        """Check if we have a recent detection."""
        if self.last_detection_time is None:
            return False
        elapsed = (self.get_clock().now() -
                   self.last_detection_time).nanoseconds / 1e9
        return elapsed < self.detection_timeout

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
        """Center object in frame. Returns True if centered."""
        obj_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)
        image_center_x = self.image_width / 2.0
        offset_x = obj_center_x - image_center_x

        if abs(offset_x) <= self.center_tolerance:
            self.stop_base()
            return True

        angular_ratio = -offset_x / (self.image_width / 2.0)
        angular_ratio = max(-1.0, min(1.0, angular_ratio))
        angular_vel = angular_ratio * self.turn_speed

        self.teleop_pub.set_velocity(linear_x=0.0, angular_z=angular_vel)
        return False

    def approach_object(self, yolo_data: YOLOData):
        """Move to ideal grabbing distance. Returns True if at correct distance."""
        bbox_error = yolo_data.bbox_w - self.target_bbox_width

        if abs(bbox_error) <= self.bbox_tolerance:
            self.stop_base()
            return True

        if bbox_error < -self.bbox_tolerance:
            linear_vel = self.forward_speed
        else:
            linear_vel = -self.forward_speed * 0.5

        self.teleop_pub.set_velocity(linear_x=linear_vel, angular_z=0.0)
        return False

    def send_arm_trajectory(self, positions: dict, time_sec: float = 2.0):
        """Send arm trajectory using FollowJointTrajectory action."""
        if not self.arm_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Arm controller not available!")
            return None

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

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        future = self.arm_client.send_goal_async(goal)
        self.get_logger().info(f"Sent arm trajectory: {positions}")
        return future

    def send_gripper_command(self, position: float):
        """Send gripper command using GripperCommand action."""
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
        """Calculate arm position for grabbing based on object location."""
        obj_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)
        obj_center_y = yolo_data.bbox_y + (yolo_data.bbox_h / 2.0)

        norm_x = (obj_center_x - (self.image_width / 2.0)) / \
            (self.image_width / 2.0)
        norm_y = (obj_center_y - (self.image_height / 2.0)) / \
            (self.image_height / 2.0)

        camera_hfov_rad = math.radians(62.0)
        joint1 = norm_x * (camera_hfov_rad / 2.0)
        joint1 = max(-1.5, min(1.5, joint1))

        joint2 = self.grab_joint2
        joint3 = self.grab_joint3
        joint4 = self.grab_joint4

        if norm_y > 0.2:
            joint2 += self.grab_vertical_adjust

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
        """Main state machine tick (10 Hz)."""

        # ==================== VOICE_SEARCH ====================
        if self.state == State.VOICE_SEARCH:
            # Check if bottle is detected
            if self.detection_is_fresh() and self.last_yolo_data is not None:
                self.get_logger().info("\n" + "=" * 70)
                self.get_logger().info(
                    "[VOICE_SEARCH] Bottle found! Switching to visual servoing...")
                self.get_logger().info("=" * 70)
                self._speak("Bottle detected. Locking on.")
                self.state = State.CENTERING
                return

            # Check for new audio command from topic
            if self.last_audio_command is not None:
                text = self.last_audio_command
                self.last_audio_command = None  # Clear after processing

                cmd = self._parse_command(text)

                if cmd != 'unknown':
                    self._execute_voice_command(cmd)
                else:
                    self._speak(
                        "Command not recognized. Try turn left, turn right, move forward, or scan.")

            # Prompt for voice command periodically (visual feedback only)
            now = self.get_clock().now()
            if self.last_prompt_time is None:
                self.last_prompt_time = now

            elapsed = (now - self.last_prompt_time).nanoseconds / 1e9
            if elapsed >= self.prompt_delay:
                self._speak("Ready for Command")
                self.last_prompt_time = now

        # ==================== EXECUTING_CMD ====================
        elif self.state == State.EXECUTING_CMD:
            elapsed = (self.get_clock().now() -
                       self.cmd_start_time).nanoseconds / 1e9

            if elapsed >= self.cmd_duration:
                self.stop_base()
                self.scanning = False
                self.get_logger().info("[EXECUTING_CMD] Command complete")
                self.state = State.VOICE_SEARCH
                self.last_prompt_time = self.get_clock().now()  # Reset prompt timer

        # ==================== CENTERING (Part 2) ====================
        elif self.state == State.CENTERING:
            if not self.detection_is_fresh():
                self.get_logger().warn(
                    "[CENTERING] Lost detection, returning to voice search")
                self.stop_base()
                self._speak("Lost sight of bottle. Ready for Command.")
                self.state = State.VOICE_SEARCH
                return

            if self.center_on_object(self.last_yolo_data):
                self.get_logger().info(
                    "[CENTERING] Centered! Moving to APPROACHING...")
                self.state = State.APPROACHING

        # ==================== APPROACHING (Part 2) ====================
        elif self.state == State.APPROACHING:
            if not self.detection_is_fresh():
                self.get_logger().warn(
                    "[APPROACHING] Lost detection, returning to voice search")
                self.stop_base()
                self._speak("Lost sight of bottle. Ready for Command.")
                self.state = State.VOICE_SEARCH
                return

            if not self.is_centered(self.last_yolo_data):
                self.center_on_object(self.last_yolo_data)
                return

            if self.approach_object(self.last_yolo_data):
                self.get_logger().info(
                    "[APPROACHING] At grab distance! Starting GRABBING...")
                self.stop_base()
                self._speak("Grabbing bottle")
                self.grab_step = 0
                self.state = State.GRABBING

        # ==================== GRABBING (Part 2) ====================
        elif self.state == State.GRABBING:
            self._execute_grab_sequence()

        # ==================== TRANSPORTING (Part 2) ====================
        elif self.state == State.TRANSPORTING:
            if self.transport_start_time is None:
                self.transport_duration = self.transport_distance / self.forward_speed
                self.transport_start_time = self.get_clock().now()

                self.get_logger().info(
                    f"[TRANSPORTING] Moving forward {self.transport_distance}m")
                self._speak("Transporting bottle")
                self.teleop_pub.set_velocity(
                    linear_x=self.forward_speed, angular_z=0.0)
            else:
                elapsed = (self.get_clock().now() -
                           self.transport_start_time).nanoseconds / 1e9

                if elapsed >= self.transport_duration:
                    self.stop_base()
                    self.get_logger().info(
                        "[TRANSPORTING] Complete! Starting RELEASING...")
                    self.release_step = 0
                    self.state = State.RELEASING

        # ==================== RELEASING (Part 2) ====================
        elif self.state == State.RELEASING:
            self._execute_release_sequence()

        # ==================== DONE ====================
        elif self.state == State.DONE:
            self.get_logger().info("\n" + "=" * 70)
            self.get_logger().info("MISSION COMPLETE!")
            self.get_logger().info("Bottle successfully retrieved and transported!")
            self.get_logger().info("=" * 70 + "\n")

            self._speak("Mission complete")

            # Reset for next mission
            self.last_yolo_data = None
            self.last_detection_time = None
            self.transport_start_time = None
            self.state = State.VOICE_SEARCH
            self.last_prompt_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # Part 2 Sequences
    # ------------------------------------------------------------------
    def _execute_grab_sequence(self):
        """Multi-step grabbing sequence (from Part 2)."""

        if self.grab_step == 0:
            self.get_logger().info("[GRAB] Step 0: Opening gripper...")
            self.gripper_action_future = self.send_gripper_command(
                self.gripper_open)
            self.step_start_time = self.get_clock().now()
            self.grab_step = 1

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

        elif self.grab_step == 2:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 3.0:
                self.get_logger().info("[GRAB] Step 2: Extending to grasp...")
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

        elif self.grab_step == 3:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.get_logger().info("[GRAB] Step 3: Closing gripper...")
                self.gripper_action_future = self.send_gripper_command(
                    self.gripper_close)
                self.step_start_time = self.get_clock().now()
                self.grab_step = 4

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

        elif self.grab_step == 5:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.5:
                self.get_logger().info(
                    "[GRAB] Object grabbed! Moving to TRANSPORTING...")
                self.grab_step = 0
                self.state = State.TRANSPORTING

    def _execute_release_sequence(self):
        """Multi-step release sequence (from Part 2)."""

        if self.release_step == 0:
            self.get_logger().info("[RELEASE] Step 0: Lowering arm...")
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

        elif self.release_step == 1:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.5:
                self.get_logger().info("[RELEASE] Step 1: Opening gripper...")
                self.gripper_action_future = self.send_gripper_command(
                    self.gripper_open)
                self.step_start_time = self.get_clock().now()
                self.release_step = 2

        elif self.release_step == 2:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.get_logger().info(
                    "[RELEASE] Step 2: Retracting to home...")
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

        elif self.release_step == 3:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.5:
                self.get_logger().info("[RELEASE] Release complete!")
                self.release_step = 0
                self.state = State.DONE

    def shutdown(self):
        """Clean shutdown."""
        self.get_logger().info("Shutting down Part 3...")
        self.stop_base()
        self.timer.cancel()
        self.teleop_pub.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Part3()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down Part 3...")
    except Exception as e:
        print(f"Error in Part 3: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
