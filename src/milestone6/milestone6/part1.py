#!/usr/bin/env python3
"""
Part 1: Visual Servoing (Centering Only)

This node implements the full Part 1 requirements:
1. Detect bottle using YOLO
2. Center on the bottle (rotate base to align)

Usage:
    ros2 run milestone6 part1
"""
import rclpy

from milestone6.robot import Robot
from milestone6.yolo.yolo_data import YOLOData


class Part1(Robot):
    """
    Part 1: Visual Servoing (Centering Only).

    Implements visual servoing to continuously track and center on a detected bottle.
    The robot rotates to keep the target object centered in the camera frame.
    Does NOT approach the object - only maintains centered alignment.
    """

    def __init__(self):
        super().__init__('part1')
        self._is_moving = False
        self.info("Initialized Part 1: Visual Servoing (Centering Only).")
        self.info(f"Parameters: {self.params}")

    def _tick(self):
        """Main control loop - center on object if detected."""
        if self.last_yolo_data is not None and self.detection_is_fresh():
            self.center_on_object(self.last_yolo_data)
        elif self._is_moving:
            # Target lost, stop movement
            self.stop_movement()
            self._is_moving = False

    def _yolo_callback(self, data: YOLOData):
        """Process YOLO detection and update last detection data."""
        super()._yolo_callback(data)
        self._is_moving = True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Part1()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in Part 1: {e}")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
