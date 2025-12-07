# Code taken from:
# https://github.com/ros2/examples/blob/fa10c22610648a90e7344cff4c27cd3356837543/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py#L1C1-L51C11
#
# Copyright 2016 Open Source Robotics Foundation, Inc.
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
from std_msgs.msg import String

import json

from .yolo_data import YOLOData


class YOLOSubscriber:
    """
    Subscribes the following information:

        - Bounding box pixel location
        - Bounding box pixel width
        - Bounding box pixel height
        - Class information
        - Publishing the camera image is optional.

    Data Types: https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
    """

    def __init__(self, node, callback=None):
        
        self._node = node
        self._callback = callback
        self._subscription = node.create_subscription(
            String,
            'yolo_topic',
            self._listener_callback,
            10
        )

    def _listener_callback(self, msg):
        try:
            data = YOLOData(**json.loads(msg.data))
            self._node.get_logger().info(f"Received: {str(data)}")
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to decode JSON: {msg.data}")
        
        return data