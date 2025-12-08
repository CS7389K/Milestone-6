#!/usr/bin/env python3
import json
import threading
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# You created this in Step 3
from milestone6.api.arm_sequences import ArmSequences


class MissionState(Enum):
    SEARCH_ALIGN = 1
    APPROACH = 2
    GRAB = 3
    GRAB_IN_PROGRESS = 4  # New state to wait for grab to complete
    MOVE_FORWARD_1M = 5
    RELEASE = 6
    RELEASE_IN_PROGRESS = 7  # New state to wait for release to complete
    DONE = 8


class Part2Mission(Node):
    def __init__(self):
        super().__init__('part2_mission')

        # ------------------- Parameters (safe defaults) -------------------
        # YOLOPublisher default image width=500, height=320
        self.declare_parameter('image_width', 500)
        self.declare_parameter('align_tol_px', 20)
        self.declare_parameter('ang_k', 0.002)
        self.declare_parameter('approach_v', 0.05)
        self.declare_parameter('close_bbox_w', 160)
        self.declare_parameter('bottle_timeout_sec', 0.5)

        self.declare_parameter('move_speed', 0.08)
        self.declare_parameter('move_dist', 1.0)

        # Optional class filter:
        # set to -1 to accept any class
        self.declare_parameter('target_class', -1)

        self.image_width = float(self.get_parameter('image_width').value)
        self.img_cx = self.image_width / 2.0

        self.align_tol_px = float(self.get_parameter('align_tol_px').value)
        self.ang_k = float(self.get_parameter('ang_k').value)
        self.approach_v = float(self.get_parameter('approach_v').value)
        self.close_bbox_w = float(self.get_parameter('close_bbox_w').value)
        self.bottle_timeout_sec = float(self.get_parameter('bottle_timeout_sec').value)

        self.move_speed = float(self.get_parameter('move_speed').value)
        self.move_dist = float(self.get_parameter('move_dist').value)
        self.move_time = self.move_dist / self.move_speed if self.move_speed > 0 else 0.0

        self.target_class = int(self.get_parameter('target_class').value)

        # ------------------- Publishers/Subscribers -------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Your YOLO publisher:
        # self._publisher = node.create_publisher(String, 'yolo_topic', 10)
        self.yolo_sub = self.create_subscription(
            String, 'yolo_topic', self.on_yolo, 10
        )

        # ------------------- Arm sequences -------------------
        self.arm = ArmSequences(self)

        # ------------------- Detection cache -------------------
        # YOLO publisher sends one detection per message.
        # We'll keep the best (widest) box within recent time window.
        self.best_bbox = None  # dict
        self.last_seen_time = None

        # ------------------- State machine -------------------
        self.state = MissionState.SEARCH_ALIGN
        self.state_entered = True

        # MOVE state bookkeeping (non-blocking)
        self.move_start_time = None

        # ------------------- Threading flags -------------------
        self.arm_action_complete = False
        self.arm_action_failed = False
        self.arm_thread = None

        # Timer tick
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

        self.get_logger().info("Part2Mission ready: ALIGN->APPROACH->GRAB->MOVE_1M->RELEASE")


    # ------------------------------------------------------------------
    # YOLO callback
    # ------------------------------------------------------------------
    def on_yolo(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return

        # Required fields from your publisher
        if not all(k in d for k in ('bbox_x', 'bbox_y', 'bbox_w', 'bbox_h', 'clz')):
            return

        # Optional class filtering
        clz = int(d['clz'])
        if self.target_class != -1 and clz != self.target_class:
            return

        # Keep the largest bbox_w as current target
        if self.best_bbox is None or float(d['bbox_w']) > float(self.best_bbox['bbox_w']):
            self.best_bbox = d
            self.last_seen_time = self.get_clock().now()


    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def set_state(self, new_state: MissionState):
        if new_state != self.state:
            self.state = new_state
            self.state_entered = True
            # Reset per-state variables if needed
            if new_state == MissionState.MOVE_FORWARD_1M:
                self.move_start_time = self.get_clock().now()


    def stop_base(self):
        self.cmd_pub.publish(Twist())


    def bottle_is_fresh(self):
        if self.best_bbox is None or self.last_seen_time is None:
            return False
        age = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9
        return age <= self.bottle_timeout_sec


    def get_error_and_close(self):
        """
        Returns:
            (error_px, close_enough_bool)
        """
        x = float(self.best_bbox['bbox_x'])
        w = float(self.best_bbox['bbox_w'])

        bbox_cx = x + w / 2.0
        error = bbox_cx - self.img_cx

        close_enough = w >= self.close_bbox_w
        return error, close_enough


    def publish_align(self):
        """
        Rotate to center bottle.
        """
        error, _ = self.get_error_and_close()
        aligned = abs(error) < self.align_tol_px

        t = Twist()
        t.linear.x = 0.0
        t.angular.z = -self.ang_k * error
        self.cmd_pub.publish(t)

        return aligned


    def publish_approach(self):
        """
        Approach slowly, only forward when aligned.
        """
        error, close_enough = self.get_error_and_close()
        aligned = abs(error) < self.align_tol_px

        t = Twist()
        t.angular.z = -self.ang_k * error

        if aligned and not close_enough:
            t.linear.x = self.approach_v
        else:
            t.linear.x = 0.0

        self.cmd_pub.publish(t)
        return close_enough


    def publish_move_forward_1m(self):
        """
        Non-blocking open-loop motion.
        """
        if self.move_start_time is None:
            self.move_start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.move_start_time).nanoseconds / 1e9
        if elapsed >= self.move_time:
            self.stop_base()
            return True

        t = Twist()
        t.linear.x = self.move_speed
        t.angular.z = 0.0
        self.cmd_pub.publish(t)
        return False


    # ------------------------------------------------------------------
    # Threaded arm action helpers
    # ------------------------------------------------------------------
    def _grab_thread_func(self):
        """Thread function to execute grab sequence."""
        try:
            self.get_logger().info("Starting GRAB sequence in thread...")
            self.arm.grab()
            self.arm_action_complete = True
            self.arm_action_failed = False
            self.get_logger().info("GRAB sequence completed successfully!")
        except Exception as e:
            self.get_logger().error(f"Grab failed: {e}")
            self.arm_action_complete = True
            self.arm_action_failed = True


    def _release_thread_func(self):
        """Thread function to execute release sequence."""
        try:
            self.get_logger().info("Starting RELEASE sequence in thread...")
            self.arm.release()
            self.arm_action_complete = True
            self.arm_action_failed = False
            self.get_logger().info("RELEASE sequence completed successfully!")
        except Exception as e:
            self.get_logger().error(f"Release failed: {e}")
            self.arm_action_complete = True
            self.arm_action_failed = True


    # ------------------------------------------------------------------
    # Main tick
    # ------------------------------------------------------------------
    def tick(self):
        # If we lose bottle during align/approach, go back to searching
        if self.state in (MissionState.SEARCH_ALIGN, MissionState.APPROACH):
            if not self.bottle_is_fresh():
                self.best_bbox = None
                self.last_seen_time = None
                self.stop_base()
                self.set_state(MissionState.SEARCH_ALIGN)
                self.state_entered = False  # already handled
                return

        # ------------------- STATE: SEARCH_ALIGN -------------------
        if self.state == MissionState.SEARCH_ALIGN:
            # If no detection, just idle
            if self.best_bbox is None:
                self.stop_base()
                if self.state_entered:
                    self.get_logger().info("Waiting for YOLO detections on 'yolo_topic'...")
                    self.state_entered = False
                return

            aligned = self.publish_align()
            if aligned:
                self.stop_base()
                self.set_state(MissionState.APPROACH)
            return

        # ------------------- STATE: APPROACH -------------------
        if self.state == MissionState.APPROACH:
            close_enough = self.publish_approach()
            if close_enough:
                self.stop_base()
                self.set_state(MissionState.GRAB)
            return

        # ------------------- STATE: GRAB -------------------
        if self.state == MissionState.GRAB:
            if self.state_entered:
                self.state_entered = False
                self.stop_base()
                self.get_logger().info("Executing GRAB sequence...")
                
                # Reset flags
                self.arm_action_complete = False
                self.arm_action_failed = False
                
                # Start grab in separate thread
                self.arm_thread = threading.Thread(
                    target=self._grab_thread_func,
                    daemon=True
                )
                self.arm_thread.start()
                
                # Transition to waiting state
                self.set_state(MissionState.GRAB_IN_PROGRESS)
            return

        # ------------------- STATE: GRAB_IN_PROGRESS -------------------
        if self.state == MissionState.GRAB_IN_PROGRESS:
            # Wait for arm action to complete
            if self.arm_action_complete:
                if self.arm_action_failed:
                    self.get_logger().error("Grab failed, returning to search")
                    self.set_state(MissionState.SEARCH_ALIGN)
                else:
                    self.get_logger().info("Grab successful, moving forward")
                    self.set_state(MissionState.MOVE_FORWARD_1M)
            else:
                # Still waiting, keep robot stopped
                self.stop_base()
            return

        # ------------------- STATE: MOVE_FORWARD_1M -------------------
        if self.state == MissionState.MOVE_FORWARD_1M:
            done = self.publish_move_forward_1m()
            if done:
                self.set_state(MissionState.RELEASE)
            return

        # ------------------- STATE: RELEASE -------------------
        if self.state == MissionState.RELEASE:
            if self.state_entered:
                self.state_entered = False
                self.stop_base()
                self.get_logger().info("Executing RELEASE sequence...")
                
                # Reset flags
                self.arm_action_complete = False
                self.arm_action_failed = False
                
                # Start release in separate thread
                self.arm_thread = threading.Thread(
                    target=self._release_thread_func,
                    daemon=True
                )
                self.arm_thread.start()
                
                # Transition to waiting state
                self.set_state(MissionState.RELEASE_IN_PROGRESS)
            return

        # ------------------- STATE: RELEASE_IN_PROGRESS -------------------
        if self.state == MissionState.RELEASE_IN_PROGRESS:
            # Wait for arm action to complete
            if self.arm_action_complete:
                if self.arm_action_failed:
                    self.get_logger().error("Release failed, but continuing to DONE")
                else:
                    self.get_logger().info("Release successful")
                self.set_state(MissionState.DONE)
            else:
                # Still waiting, keep robot stopped
                self.stop_base()
            return

        # ------------------- STATE: DONE -------------------
        if self.state == MissionState.DONE:
            self.stop_base()
            return


def main():
    rclpy.init()
    node = Part2Mission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
