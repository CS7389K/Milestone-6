#!/usr/bin/env python3
"""
Part 4: LLM-Guided Object Search and Retrieval

This node extends Part 2 by using an LLM for high-level planning instead of 
direct visual servoing start. The robot receives natural language instructions that
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
from std_msgs.msg import String

from milestone6.part2 import Part2
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
    SEARCHING = 8        # Spinning to search for specific object
    DONE = 9             # Complete


class Part4(Part2):
    """
    LLM-guided high-level planning with Part 2 visual servoing integration.
    Extends Part2 to add LLM-driven atomic actions and multi-object tracking.
    """

    # Override and extend Part2 parameters
    PARAMETERS = {
        'tracking_classes': '39,77,64',  # bottle, bear, mouse
        'turn_duration': 1.0,           # seconds
        'forward_duration': 0.5,        # seconds
        'scan_speed': 0.5,              # rad/s
        'forward_speed_llm': 0.2,       # m/s
    }

    def __init__(self):
        # Initialize Part2 (which initializes Robot base class)
        super().__init__()

        # Change node name from 'part2' to 'part3'
        self._node_name = 'part4'

        # Override the state to Part4's initial state
        self.state = State.LLM_GUIDED

        self.info("Initializing Part 4: LLM-Guided Mission...")

        # ------------------- Part 4 State -------------------
        self.current_action = None
        self.action_start_time = None
        self.scan_complete = False
        self.initial_yaw = None

        # Object tracking
        self.current_target = None      # 'bottle', 'bear', or 'mouse'
        self.held_object = None         # Track if holding bottle
        self.placement_target = None    # Target object for placement

        # Search state
        self.search_initial_yaw = None
        self.search_target = None
        self.search_object_found = False

        # ------------------- LLM Communication -------------------
        # LLM action subscriber
        self.llm_action_sub = self.create_subscription(
            String,
            '/llm_action',
            self._llm_action_callback,
            10
        )

        # Text-to-speech publisher
        self.tts_publisher = self.create_publisher(
            String, '/text_to_speech', 10)

        self.info("Part 4 initialized - LLM-Guided Mission")
        self.info(f"Tracking classes: {self.tracking_classes}")
        self.info("Waiting for LLM actions on /llm_action...")

    # ------------------------------------------------------------------
    # YOLO Callback - Override to update Part4 tracking
    # ------------------------------------------------------------------
    def _yolo_callback(self, data: YOLOData):
        """Process YOLO detections - override to support Part4's multi-object tracking."""
        # Call parent's callback to update last_yolo_data and last_detection_time
        super()._yolo_callback(data)

        # No additional processing needed - parent handles everything

    # ------------------------------------------------------------------
    # LLM Action Callback
    # ------------------------------------------------------------------
    def _llm_action_callback(self, msg: String):
        """Process atomic action from LLM."""
        action = msg.data.strip().upper()
        self.info(f"[LLM ACTION] {action}")

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
            'bear': 77,
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
                self._speak("Turning left")
                self.teleop_publisher.set_velocity(0.0, self.scan_speed)
            else:
                self.stop_movement()
                self._finish_action()

        elif action == 'TURN_RIGHT':
            if elapsed < self.turn_duration:
                self._speak("Turning right")
                self.teleop_publisher.set_velocity(0.0, -self.scan_speed)
            else:
                self.stop_movement()
                self._finish_action()

        elif action == 'MOVE_FORWARD':
            if elapsed < self.forward_duration:
                self._speak("Moving forward")
                self.teleop_publisher.set_velocity(self.forward_speed_llm, 0.0)
            else:
                self.stop_movement()
                self._finish_action()

        elif action == 'SCAN':
            self._speak("Scanning")
            self._execute_scan()

        elif action.startswith('SEARCH'):
            target = self._parse_search_target(action)
            if target:
                self.current_target = target
                self.search_target = target
                # Update tracking_classes by modifying the parameter
                # Note: tracking_classes is already a list in Robot base class
                self.tracking_classes = [self._get_class_id_for_object(target)]
                self._speak(f"Searching for {target}")
                self.info(
                    f"Searching for {target} (class {self.tracking_classes[0]})")
                # Initialize search
                self.search_initial_yaw = None
                self.search_object_found = False
                # Clear current action and transition to SEARCHING
                self.current_action = None
                self.action_start_time = None
                self.state = State.SEARCHING
            else:
                self._finish_action()

        elif action == 'GRAB':
            # Transition to visual servoing for grabbing
            if self.detection_is_fresh():
                self.grab_step = 0  # Initialize grab sequence
                self.state = State.CENTERING
                self._speak("This is mine now")
                self._finish_action()
            else:
                self._speak("No object detected, cannot grab")
                self._finish_action()

        elif action.startswith('TRANSPORT_TO'):
            target = self._parse_transport_target(action)
            if target:
                self.placement_target = target
                self.held_object = 'bottle'  # Assume we picked up bottle
                # Update tracking to search for placement target
                self.tracking_classes = [self._get_class_id_for_object(target)]
                self._speak(f"Transporting to {target}")
                self.info(
                    f"Now tracking {target} (class {self.tracking_classes[0]}) for placement")
                self.state = State.TRANSPORTING
            self._finish_action()

        elif action == 'PLACE':
            # Check if we have a target object in view
            if self.detection_is_fresh() and self.held_object:
                self.release_step = 0  # Initialize release sequence
                self.state = State.PLACING
                self._speak("Placing object")
                self._finish_action()
            else:
                self._speak("Need to locate placement target first")
                self._finish_action()

        else:
            self.warning(f"Unknown action: {action}")
            self._finish_action()

    def _execute_scan(self):
        """Execute 360° scan."""
        if self.initial_yaw is None:
            # Initialize scan
            self.initial_yaw = self.teleop_subscriber.yaw
            self.scan_complete = False
            self.info(f"Starting scan from yaw: {self.initial_yaw:.2f}")

        current_yaw = self.teleop_subscriber.yaw

        # Calculate how far we've rotated
        yaw_diff = abs(current_yaw - self.initial_yaw)
        if yaw_diff > math.pi:
            yaw_diff = 2 * math.pi - yaw_diff

        if yaw_diff >= 2 * math.pi - 0.2:  # Nearly complete rotation
            self.stop_movement()
            self.scan_complete = True
            self.initial_yaw = None
            self._speak("Scan complete")
            self._finish_action()
        else:
            # Continue rotating
            self.teleop_publisher.set_velocity(0.0, self.scan_speed)

    def _execute_search(self):
        """Execute search by spinning until object found or 360° complete."""
        self.info(f"[SEARCH] Executing search for {self.search_target}")

        # Check if object has been found during search
        if self.detection_is_fresh() and not self.search_object_found:
            # Object found!
            self.stop_movement()
            self.search_object_found = True
            self._speak("Found you")
            self.info(f"Found {self.search_target}!")
            self.state = State.LLM_GUIDED
            self.search_initial_yaw = None
            return

        # Initialize search if needed
        if self.search_initial_yaw is None:
            self.search_initial_yaw = self.teleop_subscriber.yaw
            self.info(
                f"Starting search from yaw: {self.search_initial_yaw:.2f}")

        current_yaw = self.teleop_subscriber.yaw

        # Calculate how far we've rotated
        yaw_diff = abs(current_yaw - self.search_initial_yaw)
        if yaw_diff > math.pi:
            yaw_diff = 2 * math.pi - yaw_diff

        if yaw_diff >= 2 * math.pi - 0.2:  # Nearly complete 360° rotation
            # Completed full rotation without finding object
            self.stop_movement()
            self._speak(f"Could not find {self.search_target}")
            self.info(f"Search complete - {self.search_target} not found")
            self.state = State.LLM_GUIDED
            self.search_initial_yaw = None
        else:
            # Continue rotating
            self.teleop_publisher.set_velocity(0.0, self.scan_speed)

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
    # State Machine - Override Part2's _tick
    # ------------------------------------------------------------------
    def _tick(self):
        """Main state machine tick - overrides Part2's _tick."""
        if self.state == State.LLM_GUIDED:
            # Waiting for LLM action
            pass

        elif self.state == State.EXECUTING_CMD:
            # Execute atomic action
            self._execute_atomic_action()

        elif self.state == State.SEARCHING:
            # Execute search by spinning and looking for target
            self._execute_search()

        elif self.state == State.CENTERING:
            # Center on detected object (inherited from Part2/Robot)
            if self.detection_is_fresh():
                if self.is_centered(self.last_yolo_data):
                    self.stop_movement()
                    # Check if we're holding an object (transporting) or approaching to grab
                    if self.held_object and self.placement_target:
                        # Centered on placement target while holding object
                        # Return to TRANSPORTING state to wait for PLACE command
                        self.info(
                            f"Centered on {self.placement_target}! Ready to place.")
                        self._speak(f"Centered on {self.placement_target}")
                        self.state = State.TRANSPORTING
                    else:
                        # Normal centering before grab
                        self.state = State.APPROACHING
                        self.info("Centered! Moving to approach...")
                else:
                    self.center_on_object(self.last_yolo_data)
            else:
                self.stop_movement()
                self.warning("Lost object during centering")
                # Return to appropriate state
                if self.held_object:
                    self.state = State.TRANSPORTING
                else:
                    self.state = State.LLM_GUIDED

        elif self.state == State.APPROACHING:
            # Approach to grab distance (inherited from Part2/Robot)
            if self.detection_is_fresh():
                if self.is_at_grab_distance(self.last_yolo_data):
                    self.stop_movement()
                    self.info("At grab distance! Executing grab...")
                    self.grab_step = 0
                    self.state = State.GRABBING
                else:
                    self.approach_object(self.last_yolo_data)
            else:
                self.stop_movement()
                self.warning("Lost object during approach")
                self.state = State.LLM_GUIDED

        elif self.state == State.GRABBING:
            # Use Part2's execute_grab_sequence with next_state parameter
            self.execute_grab_sequence(next_state=State.TRANSPORTING)

        elif self.state == State.TRANSPORTING:
            # Check if we've detected the placement target while holding object
            if self.held_object and self.placement_target:
                if self.detection_is_fresh():
                    # Target detected! Center on it while still holding object
                    self._speak(
                        f"Found {self.placement_target}, centering on it")
                    self.state = State.CENTERING
                # Otherwise continue accepting LLM actions while transporting

        elif self.state == State.PLACING:
            # Use Part2's execute_release_sequence with next_state parameter
            self.execute_release_sequence(next_state=State.DONE)

        elif self.state == State.DONE:
            self.stop_movement()
            # Mission complete


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
