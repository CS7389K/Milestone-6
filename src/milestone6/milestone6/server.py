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
        self.declare_parameter('turn_speed_max', 1.5)
        self.declare_parameter('display', True)
        self.declare_parameter('track_class', 39)  # Default to "bottle"

        self._image_width = self.get_parameter('image_width').value
        self._image_height = self.get_parameter('image_height').value
        self._yolo_model  = self.get_parameter('yolo_model').value
        self._tolerance = self.get_parameter('tolerance').value 
        self._bbox_threshold = self.get_parameter('min_bbox_width').value
        self._forward_speed = self.get_parameter('forward_speed').value
        self._turn_speed_max = self.get_parameter('turn_speed_max').value
        self._display = self.get_parameter('display').value
        self._track_class = self.get_parameter('track_class').value
        
        class_name = COCO_CLASSES.get(self._track_class, 'unknown')
        self.get_logger().info("Initializing ML6 Server Node...")
        self.get_logger().info(f"Tracking COCO class: '{class_name}' (ID: {self._track_class})")

        # Initialize teleop publisher (handles all movement commands)
        self.get_logger().info("Starting Teleop Publisher...")
        self.teleop = TeleopPublisher(self)
        
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
        self.get_logger().info(f"YOLO Detection - Class: {data.clz}, BBox: ({data.bbox_x:.1f}, {data.bbox_y:.1f}, {data.bbox_w:.1f}, {data.bbox_h:.1f})")

        if data.clz != self._track_class:
            self.get_logger().info(f"Ignoring class {data.clz}, looking for {self._track_class}")
            return

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
            angular_vel = angular_ratio * self._turn_speed_max
            self.get_logger().info(f"Turning: offset={offset_x:.1f}px, angular_vel={angular_vel:.3f} rad/s")
        else:
            angular_vel = 0.0
            self.get_logger().info("Object centered horizontally")

        # Determine linear velocity (move forward if object is far)
        if data.bbox_w < self._bbox_threshold:
            linear_vel = self._forward_speed
            self.get_logger().info(f"Moving forward: bbox_w={data.bbox_w:.1f}px")
        else:
            # Object is large (close), stop
            linear_vel = 0.0
            self.get_logger().info(f"Object is close, stopping: bbox_w={data.bbox_w:.1f}px")
        
        # Update teleop velocity command
        self.teleop.set_velocity(linear_x=linear_vel, angular_z=angular_vel)
    
    def shutdown(self):
        """Clean shutdown of the server."""
        self.get_logger().info("Shutting down ML6 Server...")
        self.yolo_timer.cancel()
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