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

import time
import json

from std_msgs.msg import String

import cv2
from dataclasses import dataclass
from ultralytics import YOLO


@dataclass
class FrameData:
    """Class representing captured image frame data."""
    frame: cv2.Mat
    results: list
    start: float
    end: float


class YOLOPublisher:
    """
    Publishes the following information:

        - Bounding box pixel location
        - Bounding box pixel width
        - Bounding box pixel height
        - Class information
        - Publishing the camera image is optional.

    Data Types: https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
    """
    _GSTREAMER_PIPELINE =  (
        'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), ',
        'width={image_width}, height={image_height}, framerate=30/1, format=NV12 ! ',
        'nvvidconv ! video/x-raw,format=BGRx,width=500,height=320 ! ',
        'videoconvert ! video/x-raw,format=BGR ! appsink drop=1'
    )

    def __init__(
            self,
            node,
            callback = None,
            publish_period : float = 0.5,
            yolo_model : str = "yolov11n.hef",
            image_width: int = 500,
            image_height: int = 320,
            display: bool = True,
        ):
        self._node = node
        self._callback = callback
        self._display = display
        
        self.get_logger().info("Initializing YOLO Publisher Node...")
        self._publisher = node.create_publisher(
            String, 'yolo_topic', 10
        )
        self._timer = self._node.create_timer(publish_period, self._publish_callback)

        self.get_logger().info("Opening camera...")
        pipeline = "".join(self._GSTREAMER_PIPELINE).format(
            image_width=image_width,
            image_height=image_height
        )
        self._capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self._capture.isOpened():
            raise RuntimeError("Error: Unable to open camera")

        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO(yolo_model, task="detect")
        self.frame_data = None
        
        if self._display:
            cv2.namedWindow('YOLO Detection', cv2.WINDOW_AUTOSIZE)
            self.get_logger().info("Display window created. Press 'q' to close.")

    def _publish_callback(self):
        if self.frame_data is not None:
            # Get data from frame_data.results
            data = self.frame_data.results
            # Serialize data and publish
            msg = String()
            msg.data = json.dumps(data.__dict__)
            self._publisher.publish(msg)
            self.get_logger().info(f"Publishing: {msg.data}")

    def step(self):
        ok, frame = self._capture.read()
        if not ok:
            self.frame_data = None
        else:
            start_time = time.time()
            results = self.model(frame)
            end_time = time.time()
            
            self.frame_data = FrameData(
                frame = frame,
                start = start_time,
                results = results,
                end = end_time
            )
            
            # Display frame with detections if enabled
            if self._display:
                self._display_frame(frame, results, end_time - start_time)

    def _display_frame(self, frame, results, inference_time):
        """Display frame with YOLO detections overlaid."""
        display_frame = frame.copy()
        
        # Draw detections on frame
        if len(results) > 0 and hasattr(results[0], 'boxes') and results[0].boxes is not None:
            boxes = results[0].boxes
            for box in boxes:
                # Get box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                conf = float(box.conf[0].cpu().numpy())
                cls = int(box.cls[0].cpu().numpy())
                
                # Get class name
                class_name = self.model.names[cls] if cls < len(self.model.names) else f"Class {cls}"
                
                # Draw bounding box
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Draw label with background
                label = f"{class_name}: {conf:.2f}"
                (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(display_frame, (x1, y1 - label_h - 10), (x1 + label_w, y1), (0, 255, 0), -1)
                cv2.putText(display_frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        # Add FPS info
        fps_text = f"Inference: {inference_time*1000:.1f}ms ({1/inference_time:.1f} FPS)"
        cv2.putText(display_frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Show frame
        cv2.imshow('YOLO Detection', display_frame)
        cv2.waitKey(1)  # Process events to keep window responsive
    
    def shutdown(self):
        self._capture.release()
        cv2.destroyAllWindows()

    def get_logger(self):
        return self._node.get_logger()