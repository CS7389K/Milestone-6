#!/usr/bin/env python3
"""
YOLO Debug Subscriber Node

A standalone subscriber node that logs all YOLO detection messages for debugging.
Uses debug-level logging to show detailed detection information.

Usage:
    ros2 run milestone6 yolo_debug_subscriber
    
    # Or with launch file:
    ros2 launch milestone6 yolo_debug.launch.py
"""
import rclpy
from rclpy.node import Node

from milestone6.yolo.subscriber import YOLOSubscriber
from milestone6.yolo.yolo_data import YOLOData
from milestone6.coco import COCO_CLASSES


class YOLODebugSubscriber(Node):
    """
    Debug subscriber node for YOLO detections.
    Logs all received detection data at DEBUG level.
    """
    
    def __init__(self):
        super().__init__('yolo_debug_subscriber')
        
        self.get_logger().info("Starting YOLO Debug Subscriber...")
        self.get_logger().info("Subscribing to 'yolo_topic'...")
        
        # Create YOLO subscriber
        self.yolo_subscriber = YOLOSubscriber(self, self._on_yolo_detection)
        
        # Detection counter
        self.detection_count = 0
        
        self.get_logger().info("YOLO Debug Subscriber ready!")
        self.get_logger().info("Set log level to DEBUG to see detection details:")
        self.get_logger().info("  ros2 run milestone6 yolo_debug_subscriber --ros-args --log-level DEBUG")
    
    def _on_yolo_detection(self, data: YOLOData):
        """Callback for YOLO detections - logs detection details."""
        self.detection_count += 1
        
        # Get class name
        class_name = COCO_CLASSES.get(data.clz, f'unknown({data.clz})')
        
        # Calculate bounding box center
        center_x = data.bbox_x + (data.bbox_w / 2.0)
        center_y = data.bbox_y + (data.bbox_h / 2.0)
        
        # Log detection details at DEBUG level
        self.get_logger().debug(
            f"Detection #{self.detection_count}: "
            f"class='{class_name}' (id={data.clz}), "
            f"bbox=({data.bbox_x:.1f}, {data.bbox_y:.1f}), "
            f"size=({data.bbox_w:.1f}x{data.bbox_h:.1f}), "
            f"center=({center_x:.1f}, {center_y:.1f})"
        )
        
        # Also log at INFO level every 10 detections
        if self.detection_count % 10 == 0:
            self.get_logger().info(
                f"Received {self.detection_count} detections. "
                f"Latest: {class_name}"
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = YOLODebugSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
