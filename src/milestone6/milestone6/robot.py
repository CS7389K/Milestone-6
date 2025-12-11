"""Robot base class"""

import math
from abc import ABC, abstractmethod

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from milestone6.teleop.publisher import TeleopPublisher
from milestone6.util.coco import COCO_CLASSES
from milestone6.yolo.subscriber import YOLOSubscriber
from milestone6.yolo.yolo_data import YOLOData


class Robot(Node, ABC):
    """Abstract base class for robot control with YOLO-based object tracking.

    This class provides a framework for building robot control nodes that combine
    YOLO object detection with teleoperation capabilities for the arm and base.

    Subclasses must implement:
        _tick(): Main control loop executed periodically

    Subclasses may override:
        _yolo_callback(data): Process YOLO detection data (default implementation
                              updates last_detection_time and last_yolo_data)

    Attributes:
        PARAMETERS (dict): Default parameters that can be overridden by subclasses
        image_width (int): Camera image width in pixels
        image_height (int): Camera image height in pixels
        speed (float): Linear movement speed in m/s
        turn_speed (float): Angular turning speed in rad/s
        tracking_classes (list): COCO class IDs to track
        bbox_tolerance (int): Bounding box width tolerance in pixels
        center_tolerance (int): Center position tolerance in pixels
        target_bbox_width (int): Target bounding box width for approach in pixels
        detection_timeout (float): Time before detection is considered stale in seconds
        last_detection_time: Timestamp of most recent detection
        last_yolo_data: Most recent YOLO detection data
        teleop_publisher: Publisher for movement commands
        yolo_subscriber: Subscriber for YOLO detections
    """

    PARAMETERS = {
        'image_width': 1280,            # pixels
        'image_height': 720,            # pixels
        'speed': 0.15,                  # m/s
        'turn_speed': 1.0,              # rad/s
        'tracking_classes': '39',       # Comma-separated COCO class IDs
        'bbox_tolerance': 20,           # pixels
        'center_tolerance': 30,         # pixels
        'target_bbox_width': 180,       # pixels
        'detection_timeout': 0.5,       # seconds
    }

    def _get_merged_params(self):
        """
        Merge parameters from all parent classes in MRO order.
        Child class parameters override parent class parameters.
        """
        merged = {}
        # Traverse MRO order from base to derived (so derived overwrites base)
        # Filter out object and ABC classes, then reverse to go base->derived
        classes_with_params = [
            cls for cls in reversed(self.__class__.__mro__)
            if hasattr(cls, 'PARAMETERS') and cls.PARAMETERS is not None
        ]
        for base_class in classes_with_params:
            merged.update(base_class.PARAMETERS)
        return merged

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.info(f"Initializing Robot {node_name}...")

        # Declare and set merged parameters
        params = self._get_merged_params()
        for param_name, default_value in params.items():
            self.declare_parameter(param_name, default_value)
            self.__setattr__(param_name, self.get_parameter(param_name).value)

        # Track when we last saw the target object
        self.last_detection_time = None
        self.last_yolo_data = None

        # Parse tracking classes from comma-separated string or single integer
        if isinstance(self.tracking_classes, str):
            self.tracking_classes = [int(c.strip()) for c in self.tracking_classes.split(',')
                                     if c.strip()]
        else:
            self.tracking_classes = [int(self.tracking_classes)]

        # Get class names from COCO dataset
        self.class_names = [COCO_CLASSES.get(cls, f'unknown({cls})')
                            for cls in self.tracking_classes]

        self.info(f"Tracking COCO classes: {', '.join(self.class_names)}")

        # Initialize teleop publisher (handles all movement commands)
        self.info("Starting Teleop Publisher...")
        self.teleop_publisher = TeleopPublisher(self)

        # Initialize YOLO subscriber (receives detection results)
        self.info("Starting YOLO Subscriber...")
        self.yolo_subscriber = YOLOSubscriber(self, self._yolo_callback)

        # Create timer for main control loop
        self.control_timer = self.create_timer(0.1, self._tick)

        self.info("Initialized Robot.")

    # ---------------------------------------------------------------------------- #
    #                    Methods to be implemented by subclasses                   #
    # ---------------------------------------------------------------------------- #
    @abstractmethod
    def _tick(self):
        """Main control loop tick to be implemented by subclasses."""
        pass

    # ---------------------------------------------------------------------------- #
    #                                Helper Methods                                #
    # ---------------------------------------------------------------------------- #
    # ----------------------------------- YOLO ----------------------------------- #
    def _yolo_callback(self, data: YOLOData):
        self.debug(f"Detected tracked object: {data.clz} "
                   f"(bbox: x={data.bbox_x}, y={data.bbox_y}, "
                   f"w={data.bbox_w}, h={data.bbox_h})")

        if data.clz not in self.tracking_classes:
            self.debug(f"Ignoring class {data.clz}, "
                       f"looking for {self.tracking_classes}")
            return

        # Update detection data
        self.last_detection_time = self.get_clock().now()
        self.last_yolo_data = data

    def detection_is_fresh(self):
        """Check if we have a recent detection."""
        if self.last_detection_time is None:
            return False
        now = self.get_clock().now()
        elapsed = (now - self.last_detection_time).nanoseconds / 1e9
        is_fresh = elapsed < self.detection_timeout
        if not is_fresh:
            self.debug(f"Detection stale: {elapsed:.3f}s old")
        return is_fresh

    # -------------------------- Movement Teleoperation -------------------------- #
    def stop_movement(self):
        """Stop base movement."""
        self.teleop_publisher.set_velocity(linear_x=0.0, angular_z=0.0)

    def is_centered(self, yolo_data: YOLOData):
        """Check if object is centered in frame."""
        obj_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)
        image_center_x = self.image_width / 2.0
        offset_x = abs(obj_center_x - image_center_x)
        return offset_x <= self.center_tolerance

    def center_on_object(self, yolo_data: YOLOData):
        """
        Calculate and apply turning velocity to center object.
        Returns True if centered, False otherwise.
        """
        obj_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)
        image_center_x = self.image_width / 2.0
        offset_x = obj_center_x - image_center_x

        if abs(offset_x) <= self.center_tolerance:
            self.stop_movement()
            return True

        # Calculate turning velocity (negative offset = object left = turn left)
        angular_ratio = -offset_x / (self.image_width / 2.0)
        angular_ratio = max(-1.0, min(1.0, angular_ratio))  # Clamp to [-1, 1]
        angular_vel = angular_ratio * self.turn_speed

        self.teleop_publisher.set_velocity(linear_x=0.0, angular_z=angular_vel)
        self.debug(f"Centering: offset={offset_x:.1f}px, "
                   f"angular={angular_vel:.2f} rad/s")
        return False

    def approach_object(self, yolo_data: YOLOData):
        """
        Move forward/backward to achieve ideal grabbing distance.
        Returns True if at correct distance, False otherwise.
        """
        bbox_error = yolo_data.bbox_w - self.target_bbox_width

        if abs(bbox_error) <= self.bbox_tolerance:
            self.stop_movement()
            return True

        if bbox_error < -self.bbox_tolerance:
            # Too far, move forward
            linear_vel = self.speed
            self.debug(f"Too far (bbox={yolo_data.bbox_w:.0f}px), "
                       f"moving forward")
        else:
            # Too close, move backward
            linear_vel = -self.speed * 0.5
            self.debug(f"Too close (bbox={yolo_data.bbox_w:.0f}px), "
                       f"backing up")

        self.teleop_publisher.set_velocity(linear_x=linear_vel, angular_z=0.0)
        return False

    # ----------------------- Arm and Gripper Teleoperation ---------------------- #
    def send_arm_trajectory(self, positions: dict, time_sec: float = 2.0):
        """
        Send arm trajectory using FollowJointTrajectory action.

        Args:
            positions: Dict mapping joint names to target positions (radians)
            time_sec: Time to reach target position

        Returns:
            Future for the action goal
        """
        if not self.arm_client.wait_for_server(timeout_sec=1.0):
            self.error("Arm controller not available!")
            return None

        # Create trajectory
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [positions.get(j, 0.0) for j in joint_names]
        point.time_from_start = Duration(
            sec=int(time_sec),
            nanosec=int((time_sec % 1.0) * 1e9)
        )
        trajectory.points = [point]

        # Send goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        future = self.arm_client.send_goal_async(goal)
        self.debug(f"Sent arm trajectory: {positions}")
        return future

    def send_gripper_command(self, position: float):
        """
        Send gripper command using GripperCommand action.

        Args:
            position: Gripper position (0.025 = open, -0.015 = close)

        Returns:
            Future for the action goal
        """
        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.error("Gripper controller not available!")
            return None

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = -1.0

        future = self.gripper_client.send_goal_async(goal)
        action = "Opening" if position > 0 else "Closing"
        self.info(f"{action} gripper (pos={position})")
        return future

    def calculate_grab_position(self, yolo_data: YOLOData):
        """
        Calculate arm position for grabbing based on object location.
        Uses simple visual servoing approach.
        """
        obj_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)
        obj_center_y = yolo_data.bbox_y + (yolo_data.bbox_h / 2.0)

        # Normalize positions
        norm_x = (obj_center_x - (self.image_width / 2.0)) / \
            (self.image_width / 2.0)
        norm_y = (obj_center_y - (self.image_height / 2.0)) / \
            (self.image_height / 2.0)

        # Joint 1: Pan (base rotation) - camera FOV ~62°
        camera_hfov_rad = math.radians(62.0)
        joint1 = norm_x * (camera_hfov_rad / 2.0)
        joint1 = max(-1.5, min(1.5, joint1))  # Safety limits

        # Joints 2-4: Forward reach configuration
        # These values position arm for grabbing at optimal distance
        joint2 = self.grab_joint2
        joint3 = self.grab_joint3
        joint4 = self.grab_joint4

        # Adjust reach based on vertical position
        if norm_y > 0.2:  # Object low in frame
            joint2 += self.grab_vertical_adjust  # Reach down more

        return {
            'joint1': joint1,
            'joint2': joint2,
            'joint3': joint3,
            'joint4': joint4
        }

    def is_at_grab_distance(self, yolo_data: YOLOData):
        """Check if object is at ideal grabbing distance."""
        bbox_error = abs(yolo_data.bbox_w - self.target_bbox_width)
        return bbox_error <= self.bbox_tolerance

    # ----------------------------------- Robot ---------------------------------- #
    def shutdown(self):
        """Clean shutdown."""
        self.get_logger().info(f"Shutting down node {self.get_name()}...")
        self.stop_movement()
        self.control_timer.cancel()

    # ---------------------------------- Logging --------------------------------- #
    def debug(self, msg: str):
        """Log a debug message."""
        self.get_logger().debug(msg)

    def info(self, msg: str):
        """Log an info message."""
        self.get_logger().info(msg)

    def warning(self, msg: str):
        """Log a warning message."""
        self.get_logger().warning(msg)

    def error(self, msg: str):
        """Log an error message."""
        self.get_logger().error(msg)

    # ---------------------------------------------------------------------------- #
    #                                  Properties                                  #
    # ---------------------------------------------------------------------------- #
    @property
    def parameters(self):
        """Return current merged parameters as a dictionary."""
        return self._get_merged_params()
