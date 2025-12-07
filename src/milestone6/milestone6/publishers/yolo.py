# Code taken from:
# https://github.com/ros2/examples/blob/fa10c22610648a90e7344cff4c27cd3356837543/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py#L1C1-L53C11
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

import cv2
from dataclasses import dataclass
from typing import Optional
from ultralytics import YOLO

from .yolo_data import YOLOData

import re
print('GStreamer support: %s' % re.search(r'GStreamer\:\s+(.*)', cv2.getBuildInformation()).group(1))


@dataclass
class FrameData:
    """Class representing captured image frame data."""
    frame: cv2.Mat
    results: list
    start: float
    end: float


class YOLOPublisher(Node):
    """
    Publishes the following information:

        - Bounding box pixel location
        - Bounding box pixel width
        - Bounding box pixel height
        - Class information
        - Publishing the camera image is optional.

    Data Types: https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
    """
    _GSTREAMER_PIPELINE =  'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=500, height=320, framerate=30/1, format=NV12 ! nvvidconv ! video/x-raw,format=BGRx,width=500,height=320 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1'

    def __init__(
            self,
            publish_period : float = 0.5,
            yolo_model : str = "yolov11n.hef",
            camera_index : int = 0
        ):
        super().__init__('yolo_publisher')
        self.publisher = self.create_publisher(String, 'yolo_topic', 10)
        self.timer = self.create_timer(publish_period, self.publish_callback)
        self.capture = cv2.VideoCapture(self._GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)
        if not self.capture.isOpened():
            raise RuntimeError("Error: Unable to open camera")
        self.model = YOLO(yolo_model, task="detect")
        self.frame_data = None

    def step(self):
        ok, frame = self.capture.read()
        if not ok:
            self.frame_data = None
        else:
            self.frame_data = FrameData()
            self.frame_data.frame = frame
            self.frame_data.start = time.time()
            self.frame_data.results = self.model(frame)
            self.frame_data.end = time.time()

    def display(self) -> None:
        if self.frame_data is not None:
            annotated_frame = self.frame_data.results[0].plot()
            fps = 1 / (end - start)
            cv2.putText(
                annotated_frame,
                f"FPS: {fps:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1, 
                (0, 255, 0), 
                2
            )
            cv2.imshow("YOLOv11 Desktop Demo", annotated_frame)

    def publish_callback(self):
        if self.frame_data is not None:
            # Get data from frame_data.results
            data = self.frame_data.results
            print(str(data))
            # Serialize data and publish
            msg = String()
            msg.data = json.dumps(data.__dict__)
            self.publisher.publish(msg)
            self.get_logger().info('Publishing: "%s"' % str(msg.data))

    def destroy_node(self):
        self.capture.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOPublisher()

    try:
        while rclpy.ok():
            node.step()
            node.display()
            rclpy.spin_once(node, timeout_sec=0.0)
            # Wait for key press to break loop: ESC
            if cv2.waitKey(1) == 27: 
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()