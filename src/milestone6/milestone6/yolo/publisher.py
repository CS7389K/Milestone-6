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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import numpy as np
from hailo_platform import (HEF, ConfigureParams, FormatType, HailoStreamInterface,
                            InferVStreams, InputVStreamParams, OutputVStreamParams,
                            HailoSchedulingAlgorithm, VDevice)

from .yolo_data import YOLOData
from .coco import COCO_CLASSES


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
    # Camera capture settings for Logitech Brio (or similar webcam)
    # Aggressively optimized for speed on CPU
    _CAPTURE_WIDTH = 640  # Reduced from 1280 for faster processing
    _CAPTURE_HEIGHT = 480  # Reduced from 720 for faster processing
    _CROP_FACTOR = 0.69  # Crop from 90° (Brio) to ~62° (Pi Camera v2)
    _FRAME_SKIP = 2  # Process every Nth frame (skip 2 = process every 3rd frame)

    def __init__(
            self,
            yolo_model: str = 'models/yolov11n.hef',
            image_width: int = 320,  # Reduced from 500 for faster CPU inference
            image_height: int = 192,  # Reduced from 320 (maintains 5:3 aspect ratio)
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
        image_width = self.get_parameter('image_width').value
        image_height = self.get_parameter('image_height').value
        self._display = self.get_parameter('display').value
        
        # Store target dimensions for final output
        self._target_width = image_width
        self._target_height = image_height
        
        self.get_logger().info("Initializing YOLO Publisher Node...")
        self._publisher = self.create_publisher(
            String, 'yolo_topic', 10
        )

        self.get_logger().info("Opening camera with V4L2 backend...")
        # Use video1 (Logitech Brio USB camera)
        self._capture = cv2.VideoCapture(1, cv2.CAP_V4L2)
        
        if not self._capture.isOpened():
            raise RuntimeError("Error: Unable to open camera on /dev/video1")
        
        # Set format and capture at high resolution for better quality
        self._capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._CAPTURE_WIDTH)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._CAPTURE_HEIGHT)
        self._capture.set(cv2.CAP_PROP_FPS, 30)
        
        actual_width = int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self._capture.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info("Camera opened successfully")
        self.get_logger().info(f"Capture Resolution: {actual_width}x{actual_height} @ {actual_fps}fps")
        self.get_logger().info(f"Output Resolution: {self._target_width}x{self._target_height} (cropped & resized)")

        self.get_logger().info("Loading Hailo HEF model...")
        self._init_hailo(yolo_model)
        self.get_logger().info("Hailo model ready!")
        
        if self._display:
            cv2.namedWindow('YOLO Detection', cv2.WINDOW_AUTOSIZE)
            self.get_logger().info("Display window created. Press 'q' to close.")
        
        # Frame skipping counter (Hailo is fast, less skipping needed)
        self._frame_counter = 0
        self._last_detections = []
        
    def _init_hailo(self, hef_path: str):
        """Initialize Hailo device and HEF model."""
        try:
            # Create VDevice (Hailo device manager)
            params = VDevice.create_params()
            params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
            self._vdevice = VDevice(params)
            
            # Load HEF file
            self._hef = HEF(hef_path)
            
            # Configure network group
            configure_params = ConfigureParams.create_from_hef(self._hef, interface=HailoStreamInterface.PCIe)
            self._network_group = self._vdevice.configure(self._hef, configure_params)[0]
            self._network_group_params = self._network_group.create_params()
            
            # Get input/output specs
            self._input_vstreams_params = InputVStreamParams.make(self._network_group, quantized=False, format_type=FormatType.FLOAT32)
            self._output_vstreams_params = OutputVStreamParams.make(self._network_group, quantized=False, format_type=FormatType.FLOAT32)
            
            # Get expected input shape
            input_info = self._hef.get_input_vstream_infos()[0]
            self._model_height = input_info.shape[0]
            self._model_width = input_info.shape[1]
            
            self.get_logger().info(f"Hailo model input: {self._model_width}x{self._model_height}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Hailo: {e}")
            raise

    def step(self):
        # Frame skipping for CPU performance - only run YOLO every Nth frame
        self._frame_counter += 1
        skip_inference = (self._frame_counter % self._FRAME_SKIP) != 0
        
        # Check if camera is still open
        if not self._capture.isOpened():
            self.get_logger().error("Camera is not open!")
            return
        
        t1 = time.time()
        ok, frame = self._capture.read()
        if not ok:
            self.get_logger().warn("Failed to read frame from camera")
            return
        t2 = time.time()
        
        # Crop to match Pi Camera's narrower FOV (62° vs Brio's 90°)
        # Crop factor converts 90° FOV to ~62° FOV
        height, width = frame.shape[:2]
        crop_width = int(width * self._CROP_FACTOR)
        crop_height = int(height * self._CROP_FACTOR)
        
        x_start = (width - crop_width) // 2
        y_start = (height - crop_height) // 2
        
        frame_cropped = frame[y_start:y_start+crop_height, x_start:x_start+crop_width]
        
        # Resize to target resolution (320x192 for CPU speed)
        frame_final = cv2.resize(frame_cropped, (self._target_width, self._target_height))
        t3 = time.time()
        
        # Frame skipping: Hailo is fast, but still skip some frames for safety
        if skip_inference and self._last_detections:
            # Reuse previous detection results
            detections = self._last_detections
            t4 = t3  # No inference time
        else:
            # Run Hailo inference
            detections = self._run_hailo_inference(frame_final)
            t4 = time.time()
            # Cache results for frame skipping
            self._last_detections = detections

        # Extract and publish detection data
        if detections:
            for detection in detections:
                x1, y1, x2, y2 = detection['bbox']
                cls = detection['class_id']
                
                # Create YOLOData object
                yolo_data = YOLOData(
                    bbox_x=x1,
                    bbox_y=y1,
                    bbox_w=x2 - x1,
                    bbox_h=y2 - y1,
                    clz=cls
                )
                
                # Serialize and publish
                msg = String()
                msg.data = json.dumps({
                    'bbox_x': yolo_data.bbox_x,
                    'bbox_y': yolo_data.bbox_y,
                    'bbox_w': yolo_data.bbox_w,
                    'bbox_h': yolo_data.bbox_h,
                    'clz': yolo_data.clz
                })
                self._publisher.publish(msg)
                self.get_logger().debug(f"Detected class {cls}: bbox=({x1:.0f},{y1:.0f},{x2-x1:.0f}x{y2-y1:.0f})")
        
        # Display frame with detections if enabled
        if self._display:
            inference_time = t4 - t3
            self._display_frame(frame_final, detections, inference_time)
        
        # Log timing every 30 frames
        if self._frame_counter % 30 == 0:
            total_time = t4 - t1
            yolo_time = t4 - t3
            skip_status = "SKIPPED" if skip_inference else "RAN"
            effective_fps = 1 / total_time if total_time > 0 else 0
            self.get_logger().info(
                f"Timing (ms): capture={1000*(t2-t1):.1f}, "
                f"preprocess={1000*(t3-t2):.1f}, YOLO={1000*yolo_time:.1f} ({skip_status}), "
                f"TOTAL={1000*total_time:.1f} ms ({effective_fps:.1f} FPS)"
            )

    def _run_hailo_inference(self, frame):
        """Run inference on Hailo accelerator."""
        # Resize frame to model input size
        input_frame = cv2.resize(frame, (self._model_width, self._model_height))
        
        # Convert BGR to RGB and normalize to float32 [0, 1]
        input_data = cv2.cvtColor(input_frame, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        
        # Run inference through Hailo
        with InferVStreams(self._network_group, self._input_vstreams_params, self._output_vstreams_params) as infer_pipeline:
            input_dict = {self._input_vstreams_params[0].name: input_data}
            
            with self._network_group.activate(self._network_group_params):
                infer_results = infer_pipeline.infer(input_dict)
        
        # Post-process results (YOLOv11 format)
        detections = self._postprocess_yolo(infer_results, frame.shape[1], frame.shape[0])
        return detections
    
    def _postprocess_yolo(self, infer_results, orig_width, orig_height, conf_thresh=0.25, iou_thresh=0.45):
        """Post-process YOLO output from Hailo."""
        # Get output tensor (YOLOv11 typically outputs [1, num_detections, 84+num_classes])
        output_name = list(infer_results.keys())[0]
        output = infer_results[output_name]
        
        detections = []
        
        # YOLOv11 output format: [batch, num_detections, (x, y, w, h, conf, class_probs...)]
        for detection in output[0]:  # Iterate over batch=0
            # Extract confidence and class scores
            objectness = detection[4]
            if objectness < conf_thresh:
                continue
                
            class_scores = detection[5:]
            class_id = np.argmax(class_scores)
            class_conf = class_scores[class_id]
            
            final_conf = objectness * class_conf
            if final_conf < conf_thresh:
                continue
            
            # Extract and scale bbox coordinates
            cx, cy, w, h = detection[0:4]
            
            # Convert from normalized coordinates to pixel coordinates
            x1 = int((cx - w/2) * orig_width / self._model_width)
            y1 = int((cy - h/2) * orig_height / self._model_height)
            x2 = int((cx + w/2) * orig_width / self._model_width)
            y2 = int((cy + h/2) * orig_height / self._model_height)
            
            # Clip to frame boundaries
            x1 = max(0, min(x1, orig_width))
            y1 = max(0, min(y1, orig_height))
            x2 = max(0, min(x2, orig_width))
            y2 = max(0, min(y2, orig_height))
            
            detections.append({
                'bbox': (x1, y1, x2, y2),
                'confidence': float(final_conf),
                'class_id': int(class_id)
            })
        
        # Apply NMS
        detections = self._non_max_suppression(detections, iou_thresh)
        return detections
    
    def _non_max_suppression(self, detections, iou_thresh):
        """Apply non-maximum suppression to remove overlapping boxes."""
        if not detections:
            return []
        
        # Sort by confidence
        detections = sorted(detections, key=lambda x: x['confidence'], reverse=True)
        
        keep = []
        while detections:
            best = detections.pop(0)
            keep.append(best)
            
            # Remove overlapping detections
            detections = [d for d in detections if self._iou(best['bbox'], d['bbox']) < iou_thresh]
        
        return keep
    
    def _iou(self, box1, box2):
        """Calculate intersection over union between two boxes."""
        x1_1, y1_1, x2_1, y2_1 = box1
        x1_2, y1_2, x2_2, y2_2 = box2
        
        # Intersection area
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i < x1_i or y2_i < y1_i:
            return 0.0
        
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        
        # Union area
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0

    def _display_frame(self, frame, detections, inference_time):
        """Display frame with YOLO detections overlaid."""
        display_frame = frame.copy()
        
        # Draw detections on frame
        if detections:
            for detection in detections:
                x1, y1, x2, y2 = detection['bbox']
                conf = detection['confidence']
                cls = detection['class_id']
                
                # Get class name
                class_name = COCO_CLASSES.get(cls, f"Class {cls}")
                
                # Draw bounding box
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Draw label with background
                label = f"{class_name}: {conf:.2f}"
                (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(display_frame, (x1, y1 - label_h - 10), (x1 + label_w, y1), (0, 255, 0), -1)
                cv2.putText(display_frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        # Add FPS info
        inference_fps = (1/inference_time) if inference_time > 0 else 0
        fps_text = f"Inference: {inference_time*1000:.1f}ms ({inference_fps:.1f} FPS)"
        cv2.putText(display_frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Show frame
        try:
            cv2.imshow('YOLO Detection', display_frame)
            cv2.waitKey(1)  # Process events to keep window responsive
        except cv2.error as e:
            self.get_logger().warn(f"Display error: {e}")
            self._display = False  # Disable display if window fails
    
    def shutdown(self):
        """Clean up resources."""
        self.get_logger().info("Shutting down YOLO Publisher...")
        
        # Release Hailo resources
        if hasattr(self, '_network_group'):
            try:
                self._network_group.release()
            except Exception as e:
                self.get_logger().warn(f"Error releasing network group: {e}")
        
        if hasattr(self, '_vdevice'):
            try:
                self._vdevice.release()
            except Exception as e:
                self.get_logger().warn(f"Error releasing VDevice: {e}")
        
        # Release camera capture
        if self._capture is not None and self._capture.isOpened():
            self._capture.release()
            self.get_logger().info("Camera released")
            time.sleep(0.5)
        
        # Destroy OpenCV windows
        if self._display:
            cv2.destroyAllWindows()
        self.get_logger().info("YOLO Publisher shutdown complete.")
    
    def __del__(self):
        """Destructor to ensure cleanup happens even if shutdown() isn't called."""
        try:
            if hasattr(self, '_capture') and self._capture is not None:
                if self._capture.isOpened():
                    self._capture.release()
            if hasattr(self, '_display') and self._display:
                cv2.destroyAllWindows()
        except Exception:
            pass  # Ignore errors during cleanup


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