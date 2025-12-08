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

from .teleop.publisher import TeleopPublisher
from .teleop.subscriber import TeleopSubscriber
from .teleop.pickup_controller import PickupController

from .yolo.publisher import YOLOPublisher
from .yolo.subscriber import YOLOSubscriber
from .yolo.yolo_data import YOLOData

from .coco import COCO_CLASSES


class ML6Server(Node):
    """
    ML6 Server Node for autonomous robot control.
    
    Integrates YOLO object detection with teleoperation control
    to enable autonomous navigation towards detected objects.
    
    Message Types: https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
    """
    def __init__(self):
        super().__init__('m6_server')
        self.declare_parameter('image_width', 500)
        self.declare_parameter('image_height', 320)
        self.declare_parameter('yolo_model', 'yolo11n.pt')
        self.declare_parameter('tolerance', 50)
        self.declare_parameter('min_bbox_width', 150)
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_speed', 1.5)
        self.declare_parameter('display', True)
        self.declare_parameter('track_class', 39)  # Default to "bottle"
        self.declare_parameter('enable_pickup', True)  # Enable autonomous pickup
        self.declare_parameter('putdown_after_pickup', True)  # Auto putdown after pickup

        self._image_width = self.get_parameter('image_width').value
        self._image_height = self.get_parameter('image_height').value
        self._yolo_model  = self.get_parameter('yolo_model').value
        self._tolerance = self.get_parameter('tolerance').value 
        self._bbox_threshold = self.get_parameter('min_bbox_width').value
        self._forward_speed = self.get_parameter('forward_speed').value
        self._turn_speed = self.get_parameter('turn_speed').value
        self._display = self.get_parameter('display').value
        self._track_class = self.get_parameter('track_class').value
        self._enable_pickup = self.get_parameter('enable_pickup').value
        self._putdown_after_pickup = self.get_parameter('putdown_after_pickup').value
        
        class_name = COCO_CLASSES.get(self._track_class, 'unknown')
        self.get_logger().info("Initializing ML6 Server Node...")
        self.get_logger().info(f"Tracking COCO class: '{class_name}' (ID: {self._track_class})")
        self.get_logger().info(f"Autonomous pickup: {'enabled' if self._enable_pickup else 'disabled'}")
        self.get_logger().info(f"Auto putdown: {'enabled' if self._putdown_after_pickup else 'disabled'}")

        # Initialize teleop subscriber (tracks arm joint states)
        self.get_logger().info("Starting Teleop Subscriber...")
        self.teleop_subscriber = TeleopSubscriber(self)

        # Initialize teleop publisher (handles all movement commands)
        self.get_logger().info("Starting Teleop Publisher...")
        self.teleop = TeleopPublisher(self, self.teleop_subscriber)
        
        # Initialize pickup controller (manages pickup/putdown sequences)
        self.get_logger().info("Starting Pickup Controller...")
        self.pickup_controller = PickupController(self, self.teleop)
        
        # Initialize YOLO publisher (captures and processes camera frames)
        self.get_logger().info("Starting YOLO Publisher...")
        self.yolo_publisher = YOLOPublisher(
            self,
            yolo_model=self._yolo_model,
            image_width=self._image_width,
            image_height=self._image_height,
            display=self._display
        )
        self.yolo_timer = self.create_timer(0.1, self._yolo_step_callback)
        
        # Initialize YOLO subscriber (receives detection results)
        self.get_logger().info("Starting YOLO Subscriber...")
        self.yolo_subscriber = YOLOSubscriber(self, self._yolo_callback)
        
        # Track when we last saw the target object
        self._last_detection_time = None
        self._detection_timeout = 0.5  # Stop robot if no detection for 0.5 seconds
        
        # Create timers
        self.check_timer = self.create_timer(0.1, self._check_target_lost)
        self.pickup_update_timer = self.create_timer(0.1, self._update_pickup_controller)

        self.get_logger().info("ML6 Server Node has been started.")

    def _yolo_step_callback(self):
        """Step the YOLO publisher to process next frame."""
        self.yolo_publisher.step()
    
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
        self.get_logger().debug(f"YOLO Detection - Class: {data.clz}, BBox: ({data.bbox_x:.1f}, {data.bbox_y:.1f}, {data.bbox_w:.1f}, {data.bbox_h:.1f})")

        if data.clz != self._track_class:
            self.get_logger().debug(f"Ignoring class {data.clz}, looking for {self._track_class}")
            return

        # Update detection timestamp
        self._last_detection_time = self.get_clock().now()
        self.get_logger().info(f"Detected target class {data.clz}! Processing movement...")

        # Calculate bounding box center
        bbox_center_x = data.bbox_x + (data.bbox_w / 2.0)

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
            self.get_logger().info(f"Turning: offset={offset_x:.1f}px, angular_vel={angular_vel:.3f} rad/s")
        else:
            angular_vel = 0.0
            self.get_logger().info("Object centered horizontally")

        # Determine linear velocity (move forward if object is far)
        if data.bbox_w < self._bbox_threshold:
            linear_vel = self._forward_speed
            self.get_logger().info(f"Moving forward: bbox_w={data.bbox_w:.1f}px")
            self.pickup_controller.set_approaching()
        else:
            # Object is large (close), initiate pickup sequence
            linear_vel = 0.0
            self.get_logger().info(f"Object is close: bbox_w={data.bbox_w:.1f}px")
            
            # Execute pickup sequence if enabled
            if self._enable_pickup and not self.pickup_controller.is_busy:
                self.pickup_controller.start_pickup()
        
        # Update teleop velocity command (only if not in pickup/putdown sequence)
        if not self.pickup_controller.is_busy:
            self.teleop.set_velocity(linear_x=linear_vel, angular_z=angular_vel)
    
    def _check_target_lost(self):
        """Stop robot if target hasn't been detected recently."""
        if self._last_detection_time is None:
            return  # Haven't detected anything yet
        
        time_since_detection = (self.get_clock().now() - self._last_detection_time).nanoseconds / 1e9
        if time_since_detection > self._detection_timeout:
            # Target lost, stop the robot (only if not in pickup/putdown sequence)
            if not self.pickup_controller.is_busy:
                self.teleop.set_velocity(linear_x=0.0, angular_z=0.0)
                self.get_logger().info(f"Target lost for {time_since_detection:.2f}s, stopping robot")
            self._last_detection_time = None  # Reset so we don't spam logs
    
    def _update_pickup_controller(self):
        """Update pickup controller state machine."""
        state_changed = self.pickup_controller.update()
        
        # Check if pickup just completed and auto-putdown is enabled
        if state_changed and self.pickup_controller.state == 'holding':
            self.get_logger().info("Pickup complete! Now holding object.")
            if self._putdown_after_pickup:
                self.get_logger().info("Initiating putdown sequence...")
                # Wait a moment before starting putdown
                self.create_timer(1.0, self._start_putdown_delayed, one_shot=True)
        
        # Check if putdown completed
        elif state_changed and self.pickup_controller.state == 'complete':
            self.get_logger().info("Putdown complete! Resetting to idle.")
            self.pickup_controller.reset()
    
    def _start_putdown_delayed(self):
        """Start putdown sequence after delay."""
        self.pickup_controller.start_putdown()
    
    def shutdown(self):
        """Clean shutdown of the server."""
        self.get_logger().info("Shutting down ML6 Server...")
        self.yolo_timer.cancel()
        self.check_timer.cancel()
        self.pickup_update_timer.cancel()
        self.yolo_publisher.shutdown()
        self.teleop.shutdown()


def main(args=None):
    rclpy.init(args=args)

    server = None
    try:
        server = ML6Server()
        rclpy.spin(server)
    except KeyboardInterrupt:
        print("\nShutting down ML6 Server...")
    except Exception as e:
        print(f"Error in ML6 Server: {e}")
    finally:
        if server:
            server.shutdown()
            server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()