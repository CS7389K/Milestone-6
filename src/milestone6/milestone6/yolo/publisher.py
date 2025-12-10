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
import numpy as np
from hailo_platform import HEF, VDevice, InferVStreams, InputVStreamParams, OutputVStreamParams, FormatType

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
    # Use USB webcam with V4L2 (GStreamer pipeline for Pi Camera doesn't work with USB camera)
    _CAPTURE_WIDTH = 640
    _CAPTURE_HEIGHT = 480
    
    # Hailo batch size workaround (HEF compiled with batch=640)
    _HAILO_BATCH_SIZE = 640

    def __init__(
            self,
            yolo_model: str = 'models/yolov11n.hef',
            image_width: int = 500,
            image_height: int = 320,
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
        
        self.get_logger().info("Initializing YOLO Publisher Node...")
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

        # Initialize Hailo
        self.get_logger().info("Loading Hailo HEF model...")
        self._init_hailo(yolo_model)
        self.get_logger().info("Hailo model ready!")
        
        if self._display:
            cv2.namedWindow('YOLO Detection', cv2.WINDOW_AUTOSIZE)
            self.get_logger().info("Display window created. Press 'q' to close.")

    def _init_hailo(self, hef_path: str):
        """Initialize Hailo device and load HEF model."""
        self._hef = HEF(hef_path)
        self._vdevice = VDevice()
        self._network_groups = self._vdevice.configure(self._hef)
        self._network_group = self._network_groups[0]
        
        # Get model input shape
        input_info = self._hef.get_input_vstream_infos()[0]
        shape = input_info.shape
        self._model_height, self._model_width = shape[0], shape[1]
        
        self.get_logger().warn(f"HEF batch_size={self._HAILO_BATCH_SIZE}. Replicating frames (SLOW!)")
        self.get_logger().info(f"Model input: {self._model_width}x{self._model_height}")

    def step(self):
        """Capture frame and run inference."""
        t_start = time.time()
        
        ok, frame = self._capture.read()
        if not ok:
            self.get_logger().warn("Failed to read frame")
            return
        
        # Resize to output dimensions
        frame_resized = cv2.resize(frame, (self._output_width, self._output_height))
        
        # Run Hailo inference
        detections = self._run_inference(frame_resized)
        
        t_end = time.time()
        fps = 1.0 / (t_end - t_start) if (t_end - t_start) > 0 else 0
        
        # Publish detections
        self._publish_detections(detections)
        
        # Display
        if self._display:
            self._display_frame(frame_resized, detections, fps)

    def _run_inference(self, frame):
        """Run Hailo inference on frame."""
        # Resize to model input size
        input_frame = cv2.resize(frame, (self._model_width, self._model_height))
        
        # Convert BGR to RGB (UINT8)
        input_rgb = cv2.cvtColor(input_frame, cv2.COLOR_BGR2RGB)
        
        # Replicate frame to match batch size (workaround for batch=640 HEF)
        input_batch = np.stack([input_rgb] * self._HAILO_BATCH_SIZE, axis=0)
        
        # Run inference
        with self._network_group.activate():
            input_params = InputVStreamParams.make_from_network_group(
                self._network_group, quantized=True, format_type=FormatType.UINT8
            )
            output_params = OutputVStreamParams.make_from_network_group(
                self._network_group, quantized=False, format_type=FormatType.FLOAT32
            )
            
            with InferVStreams(self._network_group, input_params, output_params) as infer_pipeline:
                input_name = list(input_params.keys())[0]
                results = infer_pipeline.infer({input_name: input_batch})
        
        # Post-process results
        return self._postprocess(results, frame.shape[1], frame.shape[0])
    
    def _postprocess(self, results, frame_w, frame_h):
        """Extract detections from Hailo output."""
        output_name = list(results.keys())[0]
        output = results[output_name]
        
        detections = []
        
        # Handle NMS post-processed output (list format)
        if isinstance(output, list) and len(output) > 0:
            # Use first batch element
            batch_detections = output[0] if len(output) > 0 else []
            
            for det in batch_detections:
                if len(det) >= 6:
                    class_id = int(det[0])
                    confidence = float(det[1])
                    x1, y1, x2, y2 = float(det[2]), float(det[3]), float(det[4]), float(det[5])
                    
                    if confidence < 0.25:
                        continue
                    
                    # Scale to frame size
                    x1 = int(x1 * frame_w / self._model_width)
                    y1 = int(y1 * frame_h / self._model_height)
                    x2 = int(x2 * frame_w / self._model_width)
                    y2 = int(y2 * frame_h / self._model_height)
                    
                    # Clip to bounds
                    x1 = max(0, min(x1, frame_w))
                    y1 = max(0, min(y1, frame_h))
                    x2 = max(0, min(x2, frame_w))
                    y2 = max(0, min(y2, frame_h))
                    
                    detections.append({
                        'class_id': class_id,
                        'confidence': confidence,
                        'bbox': (x1, y1, x2, y2)
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
    
    def _display_frame(self, frame, detections, fps):
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
        
        # Add FPS
        cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        cv2.imshow('YOLO Detection', display_frame)
        cv2.waitKey(1)
    
    def shutdown(self):
        """Clean up resources."""
        self.get_logger().info("Shutting down YOLO Publisher...")
        
        if hasattr(self, '_vdevice'):
            self._vdevice.release()
        
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