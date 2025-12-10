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

import json
import time

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO

from ..util.camera import CameraCV2, CameraGStreamer
from .yolo_data import YOLOData


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

    _GSTREAMER_PIPELINE = (
        'nvarguscamerasrc sensor-id=0 ! '
        'video/x-raw(memory:NVMM), width={image_width}, height={image_height}, framerate=30/1, format=NV12 ! '
        'nvvidconv ! video/x-raw, format=BGRx, width={image_width}, height={image_height} ! '
        'videoconvert ! video/x-raw, format=BGR ! '
        'appsink max-buffers=1 drop=true sync=false'
    )

    def __init__(
        self,
        yolo_model: str = 'yolo11n.pt',
        image_width: int = 1280,
        image_height: int = 720,
        display: bool = True,
        camera_backend: str = 'gstreamer',  # 'cv2' or 'gstreamer'
        camera_device: int = 1,
        gstreamer_pipeline: str = '',
    ):
        super().__init__('yolo_publisher')

        # Declare parameters
        self.declare_parameter('yolo_model', yolo_model)
        self.declare_parameter('image_width', image_width)
        self.declare_parameter('image_height', image_height)
        self.declare_parameter('display', display)
        self.declare_parameter('camera_backend', camera_backend)
        self.declare_parameter('camera_device', camera_device)
        self.declare_parameter('gstreamer_pipeline', gstreamer_pipeline)

        # Get parameters
        yolo_model = self.get_parameter('yolo_model').value
        self._output_width = self.get_parameter('image_width').value
        self._output_height = self.get_parameter('image_height').value
        self._display = self.get_parameter('display').value
        camera_backend = self.get_parameter('camera_backend').value
        camera_device = self.get_parameter('camera_device').value
        gstreamer_pipeline = self.get_parameter('gstreamer_pipeline').value

        self.get_logger().info(
            f"Initializing YOLO Publisher Node (CPU mode), backend={camera_backend}...")
        self._publisher = self.create_publisher(String, 'yolo_topic', 10)

        # Camera selection
        if camera_backend == 'gstreamer':
            if not gstreamer_pipeline:
                # Use requested dimensions in the pipeline
                gstreamer_pipeline = ''.join(self._GSTREAMER_PIPELINE).format(
                    image_width=self._output_width,
                    image_height=self._output_height
                )
            self.get_logger().info(
                f"Opening camera with GStreamer pipeline: {gstreamer_pipeline}")
            self._camera = CameraGStreamer(
                gstreamer_pipeline, width=self._output_width, height=self._output_height)
        else:
            self.get_logger().info(
                f"Opening camera with OpenCV device {camera_device}")
            self._camera = CameraCV2(
                device=camera_device, width=self._output_width, height=self._output_height)

        self._camera.open()
        if not self._camera.isOpened():
            raise RuntimeError("Error: Unable to open camera")
        self.get_logger().info("Camera opened successfully")

        # Initialize YOLO model with optimizations for CPU
        self.get_logger().info(f"Loading YOLO model: {yolo_model}")
        self._model = YOLO(yolo_model, task="detect")
        self.get_logger().info("YOLO model ready!")

        if self._display:
            cv2.namedWindow('YOLO Detection', cv2.WINDOW_AUTOSIZE)
            self.get_logger().info("Display window created. Press 'q' to close.")

    def step(self):
        # Check if camera is still open
        if not self._camera.isOpened():
            self.get_logger().error("Camera is not open!")
            return

        ok, frame = self._camera.read()
        if not ok:
            self.get_logger().warn("Failed to read frame from camera")
            return

        start_time = time.time()
        results = self._model(frame)
        end_time = time.time()

        # Extract and publish detection data
        if len(results) > 0 and hasattr(results[0], 'boxes') and results[0].boxes is not None:
            boxes = results[0].boxes
            if len(boxes) > 0:
                # Publish ALL detections (not just first one)
                for box in boxes:
                    x1, y1, x2, y2 = map(float, box.xyxy[0].cpu().numpy())
                    cls = int(box.cls[0].cpu().numpy())

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
                    self.get_logger().debug(
                        f"Detected class {cls}: bbox=({x1:.0f},{y1:.0f},{x2-x1:.0f}x{y2-y1:.0f})")

        # Display frame with detections if enabled (always display, even without detections)
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
                class_name = self._model.names[cls] if cls < len(
                    self._model.names) else f"Class {cls}"

                # Draw bounding box
                cv2.rectangle(display_frame, (x1, y1),
                              (x2, y2), (0, 255, 0), 2)

                # Draw label with background
                label = f"{class_name}: {conf:.2f}"
                (label_w, label_h), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(display_frame, (x1, y1 - label_h - 10),
                              (x1 + label_w, y1), (0, 255, 0), -1)
                cv2.putText(display_frame, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # Add FPS info
        fps_text = f"Inference: {inference_time*1000:.1f}ms ({1/inference_time:.1f} FPS)"
        cv2.putText(display_frame, fps_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

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

        # Release camera capture
        if self._camera is not None and self._camera.isOpened():
            self._camera.release()
            self.get_logger().info("Camera released")
            time.sleep(0.5)

        # Destroy OpenCV windows
        if self._display:
            cv2.destroyAllWindows()
        self.get_logger().info("YOLO Publisher shutdown complete.")

    def __del__(self):
        """Destructor to ensure cleanup happens even if shutdown() isn't called."""
        try:
            if hasattr(self, '_camera') and self._camera is not None:
                if self._camera.isOpened():
                    self._camera.release()
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
