#!/usr/bin/env python3
"""
Part 2: Complete Object Grab and Transport Sequence

This node implements the full Part 2 requirements:
1. Detect bottle using YOLO
2. Center on the bottle (rotate base to align)
3. Approach the bottle (move forward until close enough)
4. Grab the bottle using arm + gripper
5. Move forward 1 meter (3.2 feet)
6. Release the bottle

States:
    IDLE          -> Waiting for object detection
    CENTERING     -> Rotating base to center object in frame
    APPROACHING   -> Moving forward to optimal grab distance
    GRABBING      -> Executing arm grab sequence
    TRANSPORTING  -> Moving forward 1 meter with object
    RELEASING     -> Placing object and retracting arm
    DONE          -> Complete

Usage:
    ros2 run milestone6 part2

Parameters:
    - tracking_classes: Comma-separated COCO class IDs to track (default: '39' for bottles)
    - image_width: Camera image width (default: 500)
    - image_height: Camera image height (default: 320)
    - center_tolerance: Centering tolerance in pixels (default: 30)
    - target_bbox_width: Ideal bbox width for grabbing (default: 180)
    - bbox_tolerance: Acceptable bbox width range (default: 20)
    - forward_speed: Base forward speed m/s (default: 0.15)
    - turn_speed: Base turn speed rad/s (default: 1.0)
    - transport_distance: Distance to move with object in meters (default: 1.0)
"""

import math
from enum import Enum

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Base and Arm control
from milestone6.teleop.publisher import TeleopPublisher
from milestone6.teleop.subscriber import TeleopSubscriber
from milestone6.util.coco import COCO_CLASSES

# YOLO detection
from milestone6.yolo.subscriber import YOLOSubscriber
from milestone6.yolo.yolo_data import YOLOData


class State(Enum):
    """States for complete grab and transport sequence."""
    IDLE = 1          # Waiting for detection
    CENTERING = 2     # Rotating to center object
    APPROACHING = 3   # Moving forward to grab distance
    GRABBING = 4      # Executing grab sequence
    TRANSPORTING = 5  # Moving forward 1 meter
    RELEASING = 6     # Placing and retracting
    DONE = 7          # Complete


class Part2(Node):
    """
    Complete Part 2 implementation with object centering,
    grabbing using FollowJointTrajectory, transport, and release.
    """

    def __init__(self):
        super().__init__('part2')

        # ------------------- Parameters -------------------
        # Comma-separated COCO class IDs
        self.declare_parameter('tracking_classes', '39')
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('bbox_tolerance', 20)  # pixels
        self.declare_parameter('center_tolerance', 30)  # pixels
        self.declare_parameter('target_bbox_width', 180)  # pixels
        self.declare_parameter('forward_speed', 0.15)  # m/s
        self.declare_parameter('turn_speed', 1.0)  # rad/s
        self.declare_parameter('detection_timeout', 1)  # seconds
        self.declare_parameter('transport_distance', 1.0)  # meters

        tracking_classes = self.get_parameter('tracking_classes').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.bbox_tolerance = self.get_parameter('bbox_tolerance').value
        self.center_tolerance = self.get_parameter('center_tolerance').value
        self.target_bbox_width = self.get_parameter('target_bbox_width').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.transport_distance = self.get_parameter(
            'transport_distance').value

        # Parse tracking classes from comma-separated string or single integer
        if isinstance(tracking_classes, str):
            self.tracking_classes = [
                int(c.strip()) for c in tracking_classes.split(',') if c.strip()]
        else:
            # Handle case where parameter is already an integer
            self.tracking_classes = [int(tracking_classes)]

        # ------------------- YOLO Subscriber -------------------
        self.get_logger().info("Starting YOLO Subscriber...")
        self.yolo_subscriber = YOLOSubscriber(self, self._yolo_callback)

        # ------------------- Teleop Control -------------------
        self.get_logger().info("Starting Teleop Publisher...")
        self.teleop_pub = TeleopPublisher(self)

        self.get_logger().info("Starting Joint State Subscriber...")
        self.teleop_sub = TeleopSubscriber(self)

        # ------------------- Action Clients -------------------
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd'
        )

        # ------------------- State Machine -------------------
        self.state = State.IDLE
        self.last_detection_time = None
        self.last_yolo_data = None

        # Sub-state tracking for multi-step operations
        self.grab_step = 0
        self.release_step = 0
        self.step_start_time = None

        # Transport tracking
        self.transport_start_time = None
        self.transport_duration = None  # Calculated from distance/speed

        # Action future tracking
        self.arm_action_future = None
        self.gripper_action_future = None

        # ------------------- Timer -------------------
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz

        class_names = [COCO_CLASSES.get(
            cls, f'unknown({cls})') for cls in self.tracking_classes]
        self.get_logger().info(
            f"Tracking COCO classes: {', '.join([f'{name} (ID: {cls})' for name, cls in zip(class_names, self.tracking_classes)])}")
        self.get_logger().info("=" * 70)
        self.get_logger().info("Part 2 Ready!")
        self.get_logger().info("Place target object at edge of camera view to start.")
        self.get_logger().info("=" * 70)

    # ------------------------------------------------------------------
    # YOLO Callback
    # ------------------------------------------------------------------
    def _yolo_callback(self, data: YOLOData):
        """Process YOLO detections."""
        # Filter by tracking classes
        if data.clz not in self.tracking_classes:
            self.get_logger().debug(
                f"Ignored detection of class ID {data.clz}")
            return

        # Update detection
        self.last_detection_time = self.get_clock().now()
        self.last_yolo_data = data

        # Log detection in IDLE state
        if self.state == State.IDLE:
            obj_center_x = data.bbox_x + (data.bbox_w / 2.0)
            class_name = COCO_CLASSES.get(data.clz, f'unknown({data.clz})')
            self.get_logger().info(
                f"{class_name} detected! BBox: ({data.bbox_x:.0f}, {data.bbox_y:.0f}, "
                f"{data.bbox_w:.0f}x{data.bbox_h:.0f}), Center: {obj_center_x:.0f}px"
            )

    # ------------------------------------------------------------------
    # Helper Methods
    # ------------------------------------------------------------------
    def detection_is_fresh(self):
        """Check if we have a recent detection."""
        if self.last_detection_time is None:
            self.get_logger().debug("No detection time set")
            return False
        elapsed = (self.get_clock().now() -
                   self.last_detection_time).nanoseconds / 1e9
        is_fresh = elapsed < self.detection_timeout
        self.get_logger().debug(
            f"Detection age: {elapsed:.3f}s, fresh: {is_fresh}")
        return is_fresh

    def stop_base(self):
        """Stop base movement."""
        self.teleop_pub.set_velocity(linear_x=0.0, angular_z=0.0)

    def is_centered(self, yolo_data: YOLOData):
        """Check if object is centered in frame."""
        obj_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)
        image_center_x = self.image_width / 2.0
        offset_x = abs(obj_center_x - image_center_x)
        return offset_x <= self.center_tolerance

    def is_at_grab_distance(self, yolo_data: YOLOData):
        """Check if object is at ideal grabbing distance."""
        bbox_error = abs(yolo_data.bbox_w - self.target_bbox_width)
        return bbox_error <= self.bbox_tolerance

    def center_on_object(self, yolo_data: YOLOData):
        """
        Calculate and apply turning velocity to center object.
        Returns True if centered, False otherwise.
        """
        obj_center_x = yolo_data.bbox_x + (yolo_data.bbox_w / 2.0)
        image_center_x = self.image_width / 2.0
        offset_x = obj_center_x - image_center_x

        if abs(offset_x) <= self.center_tolerance:
            self.stop_base()
            return True

        # Calculate turning velocity (negative offset = object left = turn left)
        angular_ratio = -offset_x / (self.image_width / 2.0)
        angular_ratio = max(-1.0, min(1.0, angular_ratio))  # Clamp to [-1, 1]
        angular_vel = angular_ratio * self.turn_speed

        self.teleop_pub.set_velocity(linear_x=0.0, angular_z=angular_vel)
        self.get_logger().debug(
            f"Centering: offset={offset_x:.1f}px, angular={angular_vel:.2f} rad/s")
        return False

    def approach_object(self, yolo_data: YOLOData):
        """
        Move forward/backward to achieve ideal grabbing distance.
        Returns True if at correct distance, False otherwise.
        """
        bbox_error = yolo_data.bbox_w - self.target_bbox_width

        if abs(bbox_error) <= self.bbox_tolerance:
            self.stop_base()
            return True

        if bbox_error < -self.bbox_tolerance:
            # Too far, move forward
            linear_vel = self.forward_speed
            self.get_logger().debug(
                f"Too far (bbox={yolo_data.bbox_w:.0f}px), moving forward")
        else:
            # Too close, move backward
            linear_vel = -self.forward_speed * 0.5
            self.get_logger().debug(
                f"Too close (bbox={yolo_data.bbox_w:.0f}px), backing up")

        self.teleop_pub.set_velocity(linear_x=linear_vel, angular_z=0.0)
        return False

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
            self.get_logger().error("Arm controller not available!")
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
        self.get_logger().info(f"Sent arm trajectory: {positions}")
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
            self.get_logger().error("Gripper controller not available!")
            return None

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = -1.0

        future = self.gripper_client.send_goal_async(goal)
        action = "Opening" if position > 0 else "Closing"
        self.get_logger().info(f"{action} gripper (pos={position})")
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
        joint2 = 0.5    # Forward reach
        joint3 = -0.3   # Extend elbow
        joint4 = 0.0    # Level gripper

        # Adjust reach based on vertical position
        if norm_y > 0.2:  # Object low in frame
            joint2 += 0.2  # Reach down more

        return {
            'joint1': joint1,
            'joint2': joint2,
            'joint3': joint3,
            'joint4': joint4
        }

    # ------------------------------------------------------------------
    # State Machine
    # ------------------------------------------------------------------
    def tick(self):
        """Main state machine tick (10 Hz)."""

        # ==================== IDLE ====================
        if self.state == State.IDLE:
            if self.detection_is_fresh() and self.last_yolo_data is not None:
                self.get_logger().info("\n" + "=" * 70)
                self.get_logger().info("START: Object detected!")
                self.get_logger().info(
                    f"Current state: {self.state}, changing to CENTERING")
                self.get_logger().info("=" * 70)
                self.state = State.CENTERING
            else:
                return

        # ==================== CENTERING ====================
        if self.state == State.CENTERING:
            self.get_logger().debug("In CENTERING state, checking detection freshness...")
            if not self.detection_is_fresh():
                self.get_logger().warn("Lost detection during centering, returning to IDLE")
                self.stop_base()
                self.state = State.IDLE
                return

            self.get_logger().debug(
                "[CENTERING] Attempting to center on object...")
            if self.center_on_object(self.last_yolo_data):
                self.get_logger().info(
                    "[CENTERING] Object centered! Moving to APPROACHING...")
                self.state = State.APPROACHING

        # ==================== APPROACHING ====================
        if self.state == State.APPROACHING:
            if not self.detection_is_fresh():
                self.get_logger().warn("Lost detection during approach, returning to IDLE")
                self.stop_base()
                self.state = State.IDLE
                return

            # First ensure centered while approaching
            if not self.is_centered(self.last_yolo_data):
                self.center_on_object(self.last_yolo_data)
                return

            # Then approach to correct distance
            if self.approach_object(self.last_yolo_data):
                self.get_logger().info(
                    "[APPROACHING] At grab distance! Starting GRABBING sequence...")
                self.stop_base()
                self.grab_step = 0
                self.state = State.GRABBING

        # ==================== GRABBING ====================
        if self.state == State.GRABBING:
            self._execute_grab_sequence()

        # ==================== TRANSPORTING ====================
        if self.state == State.TRANSPORTING:
            if self.transport_start_time is None:
                # Calculate how long to move forward
                self.transport_duration = self.transport_distance / self.forward_speed
                self.transport_start_time = self.get_clock().now()

                self.get_logger().info(f"\n{'=' * 70}")
                self.get_logger().info(f"[TRANSPORTING] Moving forward {self.transport_distance}m "
                                       f"(~{self.transport_duration:.1f}s)")
                self.get_logger().info(f"{'=' * 70}")

                # Start moving forward
                self.teleop_pub.set_velocity(
                    linear_x=self.forward_speed, angular_z=0.0)
            else:
                # Check if we've moved far enough
                elapsed = (self.get_clock().now() -
                           self.transport_start_time).nanoseconds / 1e9

                if elapsed >= self.transport_duration:
                    self.stop_base()
                    self.get_logger().info(f"[TRANSPORTING] Traveled {self.transport_distance}m! "
                                           f"Starting RELEASING sequence...")
                    self.release_step = 0
                    self.state = State.RELEASING
                else:
                    remaining = self.transport_duration - elapsed
                    if int(remaining) != int(remaining + 0.1):  # Log every second
                        self.get_logger().info(
                            f"Transporting... {remaining:.0f}s remaining")

        # ==================== RELEASING ====================
        if self.state == State.RELEASING:
            self._execute_release_sequence()

        # ==================== DONE ====================
        if self.state == State.DONE:
            self.get_logger().info("\n" + "=" * 70)
            self.get_logger().info("COMPLETE!")
            self.get_logger().info("Object successfully grabbed and transported 1 meter!")
            self.get_logger().info("=" * 70 + "\n")

            # Reset for next
            self.last_yolo_data = None
            self.last_detection_time = None
            self.transport_start_time = None
            self.state = State.IDLE
            return

    def _execute_grab_sequence(self):
        """Multi-step grabbing sequence using FollowJointTrajectory."""

        # Step 0: Open gripper
        if self.grab_step == 0:
            self.get_logger().info("[GRAB] Step 0: Opening gripper...")
            self.gripper_action_future = self.send_gripper_command(
                0.025)  # Open
            self.step_start_time = self.get_clock().now()
            self.grab_step = 1

        # Step 1: Wait for gripper to open
        elif self.grab_step == 1:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.get_logger().info(
                    "[GRAB] Step 1: Moving arm to grab position...")
                grab_pos = self.calculate_grab_position(self.last_yolo_data)
                self.arm_action_future = self.send_arm_trajectory(
                    grab_pos, time_sec=2.5)
                self.step_start_time = self.get_clock().now()
                self.grab_step = 2

        # Step 2: Wait for arm to position
        elif self.grab_step == 2:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 3.0:  # Give extra time for trajectory
                self.get_logger().info("[GRAB] Step 2: Extending to grasp...")
                # Extend slightly more to grasp
                current = self.teleop_sub.joint_positions
                grasp_pos = {
                    'joint1': current.get('joint1', 0.0),
                    'joint2': current.get('joint2', 0.5),
                    'joint3': min(1.5, current.get('joint3', -0.3) + 0.2),
                    'joint4': current.get('joint4', 0.0)
                }
                self.arm_action_future = self.send_arm_trajectory(
                    grasp_pos, time_sec=1.5)
                self.step_start_time = self.get_clock().now()
                self.grab_step = 3

        # Step 3: Wait for extension, then close gripper
        elif self.grab_step == 3:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.get_logger().info("[GRAB] Step 3: Closing gripper...")
                # Close
                self.gripper_action_future = self.send_gripper_command(-0.015)
                self.step_start_time = self.get_clock().now()
                self.grab_step = 4

        # Step 4: Wait for gripper, then lift
        elif self.grab_step == 4:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.get_logger().info("[GRAB] Step 4: Lifting object...")
                lift_pos = {
                    'joint1': self.teleop_sub.joint_positions.get('joint1', 0.0),
                    'joint2': -0.5,  # Lift up
                    'joint3': 0.4,   # Retract
                    'joint4': 0.6    # Adjust wrist
                }
                self.arm_action_future = self.send_arm_trajectory(
                    lift_pos, time_sec=2.0)
                self.step_start_time = self.get_clock().now()
                self.grab_step = 5

        # Step 5: Wait for lift to complete
        elif self.grab_step == 5:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.5:
                self.get_logger().info(
                    "[GRAB] Object grabbed! Moving to TRANSPORTING...")
                self.grab_step = 0
                self.state = State.TRANSPORTING

    def _execute_release_sequence(self):
        """Multi-step release sequence."""

        # Step 0: Lower arm
        if self.release_step == 0:
            self.get_logger().info(
                "[RELEASE] Step 0: Lowering arm to place object...")
            lower_pos = {
                'joint1': 0.0,
                'joint2': 0.3,   # Lower down
                'joint3': -0.2,  # Extend
                'joint4': 0.0
            }
            self.arm_action_future = self.send_arm_trajectory(
                lower_pos, time_sec=2.0)
            self.step_start_time = self.get_clock().now()
            self.release_step = 1

        # Step 1: Wait for lowering, then open gripper
        elif self.release_step == 1:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.5:
                self.get_logger().info(
                    "[RELEASE] Step 1: Opening gripper to release...")
                self.gripper_action_future = self.send_gripper_command(
                    0.025)  # Open
                self.step_start_time = self.get_clock().now()
                self.release_step = 2

        # Step 2: Wait for gripper, then retract arm
        elif self.release_step == 2:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.get_logger().info(
                    "[RELEASE] Step 2: Retracting arm to home position...")
                home_pos = {
                    'joint1': 0.0,
                    'joint2': -1.05,
                    'joint3': 0.35,
                    'joint4': 0.70
                }
                self.arm_action_future = self.send_arm_trajectory(
                    home_pos, time_sec=2.0)
                self.step_start_time = self.get_clock().now()
                self.release_step = 3

        # Step 3: Wait for retraction to complete
        elif self.release_step == 3:
            elapsed = (self.get_clock().now() -
                       self.step_start_time).nanoseconds / 1e9
            if elapsed >= 2.5:
                self.get_logger().info("[RELEASE] Release complete!")
                self.release_step = 0
                self.state = State.DONE

    def shutdown(self):
        """Clean shutdown."""
        self.get_logger().info("Shutting down Part 2...")
        self.stop_base()
        self.timer.cancel()
        self.teleop_pub.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Part2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error in Part 2: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
