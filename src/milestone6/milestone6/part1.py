import rclpy
from rclpy.node import Node

from milestone6.teleop.publisher import TeleopPublisher
from milestone6.util.coco import COCO_CLASSES
from milestone6.yolo.subscriber import YOLOSubscriber
from milestone6.yolo.yolo_data import YOLOData


class Part1(Node):
    """
    Part 1: Visual Servoing (Centering Only).

    Implements visual servoing to continuously track and center on a detected bottle.
    The robot rotates to keep the target object centered in the camera frame.
    Does NOT approach the object - only maintains centered alignment.

    Message Types: https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
    """

    def __init__(self):
        super().__init__('teleop_base')

        # ------------------- Parameters -------------------
        # Comma-separated COCO class IDs
        self.declare_parameter('tracking_classes', '39')
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('move_wheels', True)
        self.declare_parameter('bbox_tolerance', 20)
        self.declare_parameter('center_tolerance', 30)  # pixels
        self.declare_parameter('target_bbox_width', 180)  # pixels
        self.declare_parameter('forward_speed', 0.15)  # m/s
        self.declare_parameter('turn_speed', 1.0)  # rad/s
        self.declare_parameter('detection_timeout', 0.5)  # seconds

        tracking_classes = self.get_parameter('tracking_classes').value
        self.image_width = self.get_parameter('image_width').value
        self._move_wheels = self.get_parameter('move_wheels').value
        self.bbox_tolerance = self.get_parameter('bbox_tolerance').value
        self.center_tolerance = self.get_parameter('center_tolerance').value
        self.target_bbox_width = self.get_parameter('target_bbox_width').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.detection_timeout = self.get_parameter('detection_timeout').value

        # Parse tracking classes from comma-separated string or single integer
        if isinstance(tracking_classes, str):
            self.tracking_classes = [
                int(c.strip()) for c in tracking_classes.split(',') if c.strip()]
        else:
            # Handle case where parameter is already an integer
            self.tracking_classes = [int(tracking_classes)]

        # Get class names from COCO dataset
        class_names = [COCO_CLASSES.get(cls, f'unknown({cls})')
                       for cls in self.tracking_classes]
        class_list_str = ', '.join(
            [f'{name} (ID: {cls})' for name, cls in zip(class_names, self.tracking_classes)])

        self.get_logger().info("Initializing Part1 Node...")
        self.get_logger().info(f"Tracking COCO classes: {class_list_str}")

        # Initialize teleop publisher (handles all movement commands)
        if self._move_wheels:
            self.get_logger().info("Starting Teleop Publisher...")
            self.teleop = TeleopPublisher(self)
        else:
            self.get_logger().info("Teleop Publisher disabled by parameter 'move_wheels'.")

        # Initialize YOLO subscriber (receives detection results)
        self.get_logger().info("Starting YOLO Subscriber...")
        self.yolo_subscriber = YOLOSubscriber(self, self._yolo_callback)

        # Track when we last saw the target object
        self.last_detection_time = None
        self._is_moving = False

        # Create timer to check for lost target
        self.check_timer = self.create_timer(0.1, self._check_target_lost)

        self.get_logger().info("Part 1 Node has been started.")

    def _yolo_callback(self, data: YOLOData):
        """
        Process YOLO detection and implement visual servoing (centering only).

        Part 1 Strategy:
        - Calculate bbox center position
        - If object is left of center: turn left (positive angular.z)
        - If object is right of center: turn right (negative angular.z)
        - If object is centered: stop turning
        - NO forward/backward movement (Part 1 is centering only)

        Args:
            data: YOLOData object containing detection information
        """
        if not self._move_wheels:
            return

        # Filter by tracking classes
        if data.clz not in self.tracking_classes:
            self.get_logger().debug(
                f"Ignoring class {data.clz}, looking for {self.tracking_classes}")
            return

        # Update detection timestamp
        self.last_detection_time = self.get_clock().now()

        # Calculate bounding box center
        bbox_center_x = data.bbox_x + (data.bbox_w / 2.0)
        image_center_x = self.image_width / 2.0
        offset_x = bbox_center_x - image_center_x

        # Visual Servoing: Center the object in frame
        if abs(offset_x) <= self.center_tolerance:
            # Object is centered - stop turning
            if self._is_moving:
                self.teleop.set_velocity(linear_x=0.0, angular_z=0.0)
                self.get_logger().info(
                    f"✓ Object centered! (offset={offset_x:.1f}px, tolerance={self.center_tolerance}px)")
                self._is_moving = False
        else:
            # Object is off-center - turn to center it
            # Proportional control: turn speed proportional to offset
            # Negative offset (left) -> positive angular (turn left)
            # Positive offset (right) -> negative angular (turn right)
            angular_ratio = -offset_x / (self.image_width / 2.0)
            # Clamp to [-1, 1]
            angular_ratio = max(-1.0, min(1.0, angular_ratio))
            angular_vel = angular_ratio * self.turn_speed

            self.teleop.set_velocity(linear_x=0.0, angular_z=angular_vel)
            direction = "left" if offset_x < 0 else "right"
            self.get_logger().debug(
                f"Centering: object {direction} by {abs(offset_x):.1f}px, turning at {angular_vel:.2f} rad/s")
            self._is_moving = True

    def _check_target_lost(self):
        """Stop robot if target hasn't been detected recently."""
        if self.last_detection_time is None:
            return  # Haven't detected anything yet

        time_since_detection = (self.get_clock().now(
        ) - self.last_detection_time).nanoseconds / 1e9
        if time_since_detection > self.detection_timeout:
            # Target lost, stop the robot
            if self._is_moving:
                self.teleop.set_velocity(linear_x=0.0, angular_z=0.0)
                self.get_logger().warn(
                    f"Target lost for {time_since_detection:.2f}s, stopping robot")
                self._is_moving = False
            self.last_detection_time = None  # Reset so we don't spam logs

    def shutdown(self):
        """Clean shutdown of the server."""
        self.get_logger().debug("Shutting down Part 1...")
        self.check_timer.cancel()
        if self._move_wheels:
            self.teleop.shutdown()


def main(args=None):
    rclpy.init(args=args)

    server = None
    try:
        server = Part1()
        rclpy.spin(server)
    except KeyboardInterrupt:
        print("\nShutting down Part 1...")
    except Exception as e:
        print(f"Error in Part 1: {e}")
    finally:
        if server:
            server.shutdown()
            server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
