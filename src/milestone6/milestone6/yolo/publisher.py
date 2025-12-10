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
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
from ultralytics import YOLO

from .yolo_data import YOLOData
from ..coco import COCO_CLASSES

print('GStreamer support: %s' % re.search(r'GStreamer\:\s+(.*)', cv2.getBuildInformation()).group(1))



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
    # Optimized camera settings for faster CPU inference
    _CAPTURE_WIDTH = 640
    _CAPTURE_HEIGHT = 480

    def __init__(
            self,
            yolo_model: str = 'yolo11n.pt',  # Use .pt instead of .hef
            image_width: int = 320,  # Smaller for faster inference
            image_height: int = 192,  # Maintain aspect ratio
            display: bool = True,
        ):
        super().__init__('yolo_publisher')
        
        # Declare parameters
        self.declare_parameter('yolo_model', yolo_model)
        self.declare_parameter('image_width', image_width)
        self.declare_parameter('image_height', image_height)
        self.declare_parameter('display', display)
        
        # Get parameters
        yolo_model = self.get_parameter('yolo_model').value
        self._output_width = self.get_parameter('image_width').value
        self._output_height = self.get_parameter('image_height').value
        self._display = self.get_parameter('display').value
        
        self.get_logger().info("Initializing YOLO Publisher Node (CPU mode)...")
        self._publisher = self.create_publisher(String, 'yolo_topic', 10)

        # Open camera with V4L2
        self.get_logger().info("Opening camera with V4L2...")
        self._capture = cv2.VideoCapture(1, cv2.CAP_V4L2)
        
        if not self._capture.isOpened():
            raise RuntimeError("Error: Unable to open camera on /dev/video1")
        
        self._capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._CAPTURE_WIDTH)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._CAPTURE_HEIGHT)
        
        self.get_logger().info("Camera opened successfully")

        # Initialize YOLO model with optimizations for CPU
        self.get_logger().info(f"Loading YOLO model: {yolo_model}")
        self._model = YOLO(yolo_model, task="detect")
        
        # Warmup inference
        self.get_logger().info("Warming up model...")
        import numpy as np
        dummy_frame = np.zeros((self._output_height, self._output_width, 3), dtype=np.uint8)
        self._model(dummy_frame, imgsz=self._output_width, verbose=False)
        
        self.get_logger().info("YOLO model ready!")
        
        if self._display:
            cv2.namedWindow('YOLO Detection', cv2.WINDOW_AUTOSIZE)
            self.get_logger().info("Display window created. Press 'q' to close.")

    def step(self):
        """Capture frame and run inference."""
        t_start = time.time()
        
        ok, frame = self._capture.read()
        if not ok:
            self.get_logger().warn("Failed to read frame")
            return
        
        # Resize to output dimensions (smaller = faster)
        frame_resized = cv2.resize(frame, (self._output_width, self._output_height))
        
        # Run YOLO inference with optimizations
        t_infer_start = time.time()
        results = self._model(
            frame_resized,
            imgsz=self._output_width,  # Use frame size directly (no upscaling)
            conf=0.25,  # Confidence threshold
            iou=0.45,  # NMS IoU threshold
            verbose=False,  # Disable logging
            half=False,  # Use FP32 (FP16 can be unstable on CPU)
            device='cpu'  # Explicit CPU usage
        )
        t_infer_end = time.time()
        
        # Extract detections
        detections = self._extract_detections(results[0])
        
        t_end = time.time()
        total_fps = 1.0 / (t_end - t_start) if (t_end - t_start) > 0 else 0
        infer_time = t_infer_end - t_infer_start
        
        # Publish detections
        self._publish_detections(detections)
        
        # Display
        if self._display:
            self._display_frame(frame_resized, detections, total_fps, infer_time)

    def _extract_detections(self, result):
        """Extract detections from YOLO result."""
        detections = []
        
        if result.boxes is not None and len(result.boxes) > 0:
            boxes = result.boxes.xyxy.cpu().numpy()  # x1, y1, x2, y2
            confidences = result.boxes.conf.cpu().numpy()
            class_ids = result.boxes.cls.cpu().numpy().astype(int)
            
            for box, conf, class_id in zip(boxes, confidences, class_ids):
                x1, y1, x2, y2 = box
                detections.append({
                    'class_id': int(class_id),
                    'confidence': float(conf),
                    'bbox': (int(x1), int(y1), int(x2), int(y2))
                })
        
        return detections
    
    def _publish_detections(self, detections):
        """Publish detections to ROS topic."""
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            
            yolo_data = YOLOData(
                bbox_x=x1,
                bbox_y=y1,
                bbox_w=x2 - x1,
                bbox_h=y2 - y1,
                clz=det['class_id']
            )
            
            msg = String()
            msg.data = json.dumps({
                'bbox_x': yolo_data.bbox_x,
                'bbox_y': yolo_data.bbox_y,
                'bbox_w': yolo_data.bbox_w,
                'bbox_h': yolo_data.bbox_h,
                'clz': yolo_data.clz
            })
            self._publisher.publish(msg)
    
    def _display_frame(self, frame, detections, fps, infer_time):
        """Display frame with detections."""
        display_frame = frame.copy()
        
        # Draw detections
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            class_id = det['class_id']
            conf = det['confidence']
            
            class_name = COCO_CLASSES.get(class_id, f"Class {class_id}")
            
            cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            label = f"{class_name}: {conf:.2f}"
            (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(display_frame, (x1, y1 - label_h - 10), (x1 + label_w, y1), (0, 255, 0), -1)
            cv2.putText(display_frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        # Add timing info
        cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(display_frame, f"Inference: {infer_time*1000:.0f}ms", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        cv2.imshow('YOLO Detection', display_frame)
        cv2.waitKey(1)
    
    def shutdown(self):
        """Clean up resources."""
        self.get_logger().info("Shutting down YOLO Publisher...")
        
        if self._capture is not None and self._capture.isOpened():
            self._capture.release()
        
        if self._display:
            cv2.destroyAllWindows()
        
        self.get_logger().info("YOLO Publisher shutdown complete.")


def main(args=None):
    rclpy.init(args=args)
    publisher = YOLOPublisher()
        
    try:
        while rclpy.ok():
            publisher.step()
            rclpy.spin_once(publisher, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\nShutting down YOLO Publisher...")
    except Exception as e:
        print(f"Error in YOLO Publisher: {e}")
    finally:
        if publisher:
            publisher.shutdown()
            publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()