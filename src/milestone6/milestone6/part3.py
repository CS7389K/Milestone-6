
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
from std_msgs.msg import String

from milestone6.part2 import Part2


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


class Part3(Part2):
    """
    Voice-guided search with Part 2 visual servoing integration.
    Inherits all grab/transport/release functionality from Part2.
    """

    PARAMETERS = {
        # Voice search parameters
        'turn_duration': 1.0,           # seconds
        'forward_duration': 0.5,        # seconds
        'scan_speed': 0.5,              # rad/s
        'prompt_delay': 30.0,           # seconds between prompts
    }

    def __init__(self):
        # Initialize Part2 (which initializes Robot base class)
        super().__init__()

        # Change node name from 'part2' to 'part3'
        self._node_name = 'part3'

        self.info("Initializing Part 3: Voice-Guided Object \
                  Search and Retrieval...")
        self.info(f"Parameters: {self.params}")

        # ------------------- Text-to-Speech Publisher -------------------
        self.info("Creating text-to-speech publisher...")
        self.tts_publisher = self.create_publisher(
            String,
            '/text_to_speech',
            10
        )
        self.info("Subscribing to /voice_transcription topic...")
        self.audio_command_sub = self.create_subscription(
            String,
            '/voice_transcription',
            self._audio_command_callback,
            10
        )

        # ------------------- Voice-Specific State -------------------
        # Override Part2's initial state to start with voice search
        self.state = State.VOICE_SEARCH

        # Voice command execution tracking
        self.cmd_start_time = None
        self.cmd_duration = 0.0
        self.scanning = False
        self.scan_start_angle = None

        # Voice prompt timing
        self.last_audio_command = None
        self.last_prompt_time = None

        self.info("Part 3 initialized and ready.")
        self.info("Starting voice-guided search phase...")

        # Give initial prompt
        self._speak("Ready for Command")

    # ---------------------------------------------------------------------------- #
    #                          Voice Interaction Methods                           #
    # ---------------------------------------------------------------------------- #
    def _speak(self, text: str):
        """Publish text to be spoken."""
        self.info(f"[TTS] {text}")
        msg = String()
        msg.data = text
        self.tts_publisher.publish(msg)

    def _audio_command_callback(self, msg: String):
        """Callback for audio commands from Whisper publisher."""
        self.last_audio_command = msg.data
        self.info(f"[AUDIO] Received command: '{msg.data}'")

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
            self.teleop_publisher.set_velocity(
                linear_x=0.0, angular_z=-self.turn_speed)
            self.cmd_duration = self.turn_duration

        elif cmd == 'turn_left':
            self._speak("Turning left")
            self.teleop_publisher.set_velocity(
                linear_x=0.0, angular_z=self.turn_speed)
            self.cmd_duration = self.turn_duration

        elif cmd == 'forward':
            self._speak("Moving forward")
            self.teleop_publisher.set_velocity(
                linear_x=self.speed, angular_z=0.0)
            self.cmd_duration = self.forward_duration

        elif cmd == 'scan':
            self._speak("Scanning")
            self.scanning = True
            self.scan_start_angle = self.teleop_sub.joint_positions.get(
                'joint1', 0.0)
            self.teleop_publisher.set_velocity(
                linear_x=0.0, angular_z=self.scan_speed)
            self.cmd_duration = (2 * math.pi) / self.scan_speed  # 360 degrees

        self.cmd_start_time = self.get_clock().now()
        self.state = State.EXECUTING_CMD

    # ---------------------------------------------------------------------------- #
    #                         Main Control Loop Override                           #
    # ---------------------------------------------------------------------------- #
    def _tick(self):
        """Override Part2's tick to handle voice search states."""

        # ==================== VOICE_SEARCH ====================
        if self.state == State.VOICE_SEARCH:
            # Check if bottle is detected
            if self.detection_is_fresh() and self.last_yolo_data is not None:
                self.info("\n" + "=" * 70)
                self.info(
                    "[VOICE_SEARCH] Bottle found! Switching to visual servoing...")
                self.info("=" * 70)
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
                    self._speak("Command not recognized.")

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
                self.stop_movement()
                self.scanning = False
                self.info("[EXECUTING_CMD] Command complete")
                self.state = State.VOICE_SEARCH
                self.last_prompt_time = self.get_clock().now()  # Reset prompt timer

        # ==================== CENTERING (Part 2) ====================
        elif self.state == State.CENTERING:
            if not self.detection_is_fresh():
                self.warning(
                    "[CENTERING] Lost detection, returning to voice search")
                self.stop_movement()
                self._speak("Lost sight of bottle. Ready for Command.")
                self.state = State.VOICE_SEARCH
                return

            if self.center_on_object(self.last_yolo_data):
                self.info(
                    "[CENTERING] Centered! Moving to APPROACHING...")
                self.state = State.APPROACHING

        # ==================== APPROACHING (Part 2) ====================
        elif self.state == State.APPROACHING:
            if not self.detection_is_fresh():
                self.warning(
                    "[APPROACHING] Lost detection, returning to voice search")
                self.stop_movement()
                self._speak("Lost sight of bottle. Ready for Command.")
                self.state = State.VOICE_SEARCH
                return

            if not self.is_centered(self.last_yolo_data):
                self.center_on_object(self.last_yolo_data)
                return

            if self.approach_object(self.last_yolo_data):
                self.info(
                    "[APPROACHING] At grab distance! Starting GRABBING...")
                self.stop_movement()
                self._speak("Grabbing bottle")
                self.grab_step = 0
                self.state = State.GRABBING

        # ==================== GRABBING (Part 2) ====================
        elif self.state == State.GRABBING:
            self.execute_grab_sequence(next_state=State.TRANSPORTING)

        # ==================== TRANSPORTING (Part 2) ====================
        elif self.state == State.TRANSPORTING:
            if self.transport_start_time is None:
                self.transport_duration = self.transport_distance / self.speed
                self.transport_start_time = self.get_clock().now()

                self.info(
                    f"[TRANSPORTING] Moving forward {self.transport_distance}m")
                self._speak("Transporting bottle")
                self.teleop_publisher.set_velocity(
                    linear_x=self.speed, angular_z=0.0)
            else:
                elapsed = (self.get_clock().now() -
                           self.transport_start_time).nanoseconds / 1e9

                if elapsed >= self.transport_duration:
                    self.stop_movement()
                    self.info(
                        "[TRANSPORTING] Complete! Starting RELEASING...")
                    self.release_step = 0
                    self.state = State.RELEASING

        # ==================== RELEASING (Part 2) ==================== 
        elif self.state == State.RELEASING:
            self.execute_release_sequence(next_state=State.DONE)

        # ==================== DONE ====================
        elif self.state == State.DONE:
            self.info("\n" + "=" * 70)
            self.info("MISSION COMPLETE!")
            self.info("Bottle successfully retrieved and transported!")
            self.info("=" * 70 + "\n")

            self._speak("Mission complete")
            self._speak("Ready for Command")

            # Reset for next mission
            self.last_yolo_data = None
            self.last_detection_time = None
            self.transport_start_time = None
            self.state = State.VOICE_SEARCH
            self.last_prompt_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Part3()
        rclpy.spin(node)
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
