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
# limitations under the License.import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from .subscribers.yolo import YOLOSubscriber


class ML6Server(Node):
    """
    Message Types: https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
    """
    def __init__(self):
        super().__init__('m6_server')
        self.yolo_subscriber = YOLOSubscriber()
        self.get_logger().info("ML6 Server Node has been started.")



def main(args=None):
    rclpy.init(args=args)

    print("Initializing ML6 Server Node...")
    server = ML6Server()
    print("ML6 Server Node Initialized.")

    rclpy.spin(server)

    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()