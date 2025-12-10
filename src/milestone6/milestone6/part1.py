# Code taken from:
# https://github.com/ros2/demos/blob/1d01c8e3d06644c0d706ef83697a68efda7d0ad4/action_tutorials/action_tutorials_py/action_tutorials_py/fibonacci_action_server.py
#
#!/usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from milestone6.teleop.publisher import TeleopPublisher
from milestone6.util.coco import COCO_CLASSES
from milestone6.yolo.subscriber import YOLOSubscriber
from milestone6.yolo.yolo_data import YOLOData


class TeleopBase(Node):
    """
    TeleopBase Node for autonomous base (wheel) control.

    Integrates YOLO object detection with teleoperation control
    to enable autonomous navigation towards detected objects.
    Uses TeleopPublisher for continuous base movement commands.

    Message Types: https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
    """

    def __init__(self):
        super().__init__('teleop_base')
        self.declare_parameter('move_wheels', True)

        self.declare_parameter('image_width', 500)
        self.declare_parameter('image_height', 320)
        self.declare_parameter('tolerance', 30)  # Reduced for better centering
        # Stop when object this wide (px)
        self.declare_parameter('min_bbox_width', 150)
        # Ideal bbox width for manipulation
        self.declare_parameter('target_bbox_width', 180)
        # Acceptable range around target
        self.declare_parameter('bbox_tolerance', 20)
        self.declare_parameter('forward_speed', 0.15)
        # Reduced for smoother turning
        self.declare_parameter('turn_speed', 1.2)
        self.declare_parameter('track_class', 39)  # Default to "bottle"

        self._move_wheels = self.get_parameter('move_wheels').value

        self._image_width = self.get_parameter('image_width').value
        self._image_height = self.get_parameter('image_height').value
        self._tolerance = self.get_parameter('tolerance').value
        self._bbox_threshold = self.get_parameter('min_bbox_width').value
        self._target_bbox = self.get_parameter('target_bbox_width').value
        self._bbox_tolerance = self.get_parameter('bbox_tolerance').value
        self._forward_speed = self.get_parameter('forward_speed').value
        self._turn_speed = self.get_parameter('turn_speed').value
        self._track_class = self.get_parameter('track_class').value

        class_name = COCO_CLASSES.get(self._track_class, 'unknown')
        self.get_logger().info("Initializing TeleopBase Node...")
        self.get_logger().info(
            f"Tracking COCO class: '{class_name}' (ID: {self._track_class})")

        # Initialize teleop publisher (handles all movement commands)
        if self._move_wheels:
            self.get_logger().info("Starting Teleop Publisher...")
            self.teleop = TeleopPublisher(self)
        else:
            self.get_logger().info("Teleop Publisher disabled by parameter 'move_wheels'.")

        # Initialize YOLO subscriber (receives detection results)
        self.get_logger().info("Starting YOLO Subscriber...")
        self.yolo_subscriber = YOLOSubscriber(self, self._yolo_callback)

        # Track when we last saw the target object
        self._last_detection_time = None
        self._detection_timeout = 0.5  # Stop robot if no detection for 0.5 seconds

        # Move-wait-update behavior: prevent oscillation
        self._last_command_time = None
        self._command_cooldown = 0.3  # Wait 300ms after each command for YOLO update
        self._is_moving = False

        # Create timer to check for lost target
        self.check_timer = self.create_timer(0.1, self._check_target_lost)

        self.get_logger().info("TeleopBase Node has been started.")

    def _yolo_callback(self, data: YOLOData):
        """
        Process YOLO detection and move robot towards detected object.

        Strategy:
        - Calculate bbox center position
        - If object is left of center: turn left (positive angular.z)
        - If object is right of center: turn right (negative angular.z)
        - If object is small (far): move forward
        - If object is large (close): stop

        Args:
            data: YOLOData object containing detection information
        """
        if self._move_wheels:
            self.get_logger().debug(
                f"YOLO Detection - Class: {data.clz}, BBox: ({data.bbox_x:.1f}, {data.bbox_y:.1f}, {data.bbox_w:.1f}, {data.bbox_h:.1f})")

            if data.clz != self._track_class:
                self.get_logger().debug(
                    f"Ignoring class {data.clz}, looking for {self._track_class}")
                return

            # Update detection timestamp
            current_time = self.get_clock().now()
            self._last_detection_time = current_time

            # Move-wait-update: Skip if we recently sent a command (waiting for new detection)
            if self._last_command_time is not None:
                time_since_command = (
                    current_time - self._last_command_time).nanoseconds / 1e9
                if time_since_command < self._command_cooldown:
                    return  # Still in cooldown, wait for more detections
            self.get_logger().debug(
                f"Detected target class {data.clz}! Processing movement...")

            # Calculate bounding box center
            bbox_center_x = data.bbox_x + (data.bbox_w / 2.0)

            # Record that we're processing this detection
            self._last_command_time = current_time

            # Calculate image center
            image_center_x = self._image_width / 2.0

            # Calculate horizontal offset from center
            offset_x = bbox_center_x - image_center_x

            # Determine angular velocity (turn towards object)
            # Positive offset (object on right) -> turn right (negative angular)
            # Negative offset (object on left) -> turn left (positive angular)
            if abs(offset_x) > self._tolerance:
                # Normalize offset to [-1, 1] range
                angular_ratio = -offset_x / (self._image_width / 2.0)
                angular_ratio = max(-1.0, min(1.0, angular_ratio))  # clamp
                angular_vel = angular_ratio * self._turn_speed
                self.get_logger().debug(
                    f"Turning: offset={offset_x:.1f}px, angular_vel={angular_vel:.3f} rad/s")
            else:
                angular_vel = 0.0
                self.get_logger().debug("Object centered horizontally")

            # Determine linear velocity (move forward/backward to maintain target distance)
            bbox_error = data.bbox_w - self._target_bbox

            if abs(bbox_error) <= self._bbox_tolerance:
                # Perfect distance - object is within target range
                linear_vel = 0.0
                self.get_logger().info(
                    f"✓ At target distance! bbox={data.bbox_w:.0f}px "
                    f"(target={self._target_bbox}±{self._bbox_tolerance}px)"
                )
                # At target distance - allow turning to center
                # angular_vel already calculated above
            elif bbox_error < -self._bbox_tolerance:
                # Object too small (too far) - move forward
                # Scale speed based on error magnitude
                distance_ratio = min(1.0, abs(bbox_error) / 100.0)
                linear_vel = self._forward_speed * distance_ratio
                # IMPORTANT: Don't turn while moving forward/backward
                angular_vel = 0.0
                self.get_logger().debug(
                    f"Moving forward: bbox={data.bbox_w:.0f}px < target={self._target_bbox}px "
                    f"(error={bbox_error:.0f}px, speed={linear_vel:.2f})"
                )
            else:
                # Object too large (too close) - move backward slowly
                linear_vel = -self._forward_speed * 0.3  # Slower backward movement
                # IMPORTANT: Don't turn while moving forward/backward
                angular_vel = 0.0
                self.get_logger().debug(
                    f"Too close, backing up: bbox={data.bbox_w:.0f}px > target={self._target_bbox}px "
                    f"(error={bbox_error:.0f}px)"
                )

            # Update teleop velocity command
            self.teleop.set_velocity(
                linear_x=linear_vel, angular_z=angular_vel)
            self._is_moving = (linear_vel != 0.0 or angular_vel != 0.0)

    def _check_target_lost(self):
        """Stop robot if target hasn't been detected recently."""
        if self._last_detection_time is None:
            return  # Haven't detected anything yet

        time_since_detection = (self.get_clock().now(
        ) - self._last_detection_time).nanoseconds / 1e9
        if time_since_detection > self._detection_timeout:
            # Target lost, stop the robot
            if self._is_moving:
                self.teleop.set_velocity(linear_x=0.0, angular_z=0.0)
                self.get_logger().debug(
                    f"Target lost for {time_since_detection:.2f}s, stopping robot")
                self._is_moving = False
            self._last_detection_time = None  # Reset so we don't spam logs
            self._last_command_time = None  # Allow immediate response when target reappears

    def shutdown(self):
        """Clean shutdown of the server."""
        self.get_logger().debug("Shutting down TeleopBase...")
        self.check_timer.cancel()
        if self._move_wheels:
            self.teleop.shutdown()


def main(args=None):
    rclpy.init(args=args)

    server = None
    try:
        server = TeleopBase()
        rclpy.spin(server)
    except KeyboardInterrupt:
        print("\nShutting down TeleopBase...")
    except Exception as e:
        print(f"Error in TeleopBase: {e}")
    finally:
        if server:
            server.shutdown()
            server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
