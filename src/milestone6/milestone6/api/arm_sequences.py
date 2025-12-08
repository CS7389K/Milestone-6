#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand, FollowJointTrajectory


class ArmSequences:
    """
    FINAL robust version for your robot:

    Arm:
      - Prefer action:  /arm_controller/follow_joint_trajectory  (if available)
      - Fallback topic: /arm_controller/joint_trajectory

    Gripper:
      - Action: /gripper_controller/gripper_cmd

    Joint names verified from your /joint_states:
      joint1, joint2, joint3, joint4
      gripper_left_joint, gripper_right_joint
    """

    # ---- Confirmed topics/nodes on your robot ----
    ARM_TRAJ_TOPIC = '/arm_controller/joint_trajectory'
    ARM_TRAJ_ACTION = '/arm_controller/follow_joint_trajectory'

    GRIPPER_ACTION = '/gripper_controller/gripper_cmd'

    # ---- Verified arm joint names ----
    ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

    # ---- Gripper tuning ----
    # Your joint_states shows both gripper fingers around 0.015 when open.
    # These positions are for the GripperCommand action interface.
    GRIPPER_OPEN_POS = 0.015
    GRIPPER_CLOSE_POS = 0.0   # if doesn't grip enough, try -0.005 or 0.005
    GRIPPER_EFFORT = 1.0

    def __init__(self, node: Node):
        self.node = node

        # Arm publisher (fallback)
        self.arm_pub = node.create_publisher(JointTrajectory, self.ARM_TRAJ_TOPIC, 10)

        # Arm action client (preferred)
        self.arm_action = ActionClient(node, FollowJointTrajectory, self.ARM_TRAJ_ACTION)

        # Gripper action client
        self.gripper_client = ActionClient(node, GripperCommand, self.GRIPPER_ACTION)

        # Don't hard-fail here; we'll check availability per call
        self.arm_action.wait_for_server(timeout_sec=1.0)
        self.gripper_client.wait_for_server(timeout_sec=1.0)

    # -------------------------
    # Arm low-level helpers
    # -------------------------

    def _arm_topic(self, positions, sec=1.5):
        """
        Publish JointTrajectory with header stamp.
        This version is more compatible with stricter controllers.
        """
        msg = JointTrajectory()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.joint_names = self.ARM_JOINTS

        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start.sec = int(sec)
        pt.time_from_start.nanosec = int((sec - int(sec)) * 1e9)

        msg.points = [pt]

        # Publish multiple times
        for _ in range(5):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.arm_pub.publish(msg)
            time.sleep(0.05)

        time.sleep(sec)

    def _wait_for_future(self, future, timeout_sec=10.0):
        """
        Wait for a future to complete without using spin_until_future_complete.
        This is thread-safe when the node is already being spun in another thread.
        """
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout_sec:
                return False
            time.sleep(0.01)  # Small sleep to avoid busy waiting
        return True

    def _arm_action_send(self, positions, sec=1.5):
        """
        Send FollowJointTrajectory goal.
        Thread-safe version that doesn't use spin_until_future_complete.
        """
        if not self.arm_action.wait_for_server(timeout_sec=2.0):
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.ARM_JOINTS

        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start.sec = int(sec)
        pt.time_from_start.nanosec = int((sec - int(sec)) * 1e9)

        goal.trajectory.points = [pt]

        send_future = self.arm_action.send_goal_async(goal)
        
        # Wait for goal to be accepted (thread-safe)
        if not self._wait_for_future(send_future, timeout_sec=5.0):
            self.node.get_logger().warn("Arm action goal send timeout")
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        
        # Wait for result (thread-safe)
        if not self._wait_for_future(result_future, timeout_sec=sec + 2.0):
            self.node.get_logger().warn("Arm action result timeout")
            return False

        time.sleep(0.1)
        return True

    def _arm(self, j1, j2, j3, j4, sec=1.5):
        positions = [j1, j2, j3, j4]

        # Prefer action if available
        ok = self._arm_action_send(positions, sec=sec)
        if ok:
            return

        # Fallback to topic
        self._arm_topic(positions, sec=sec)

    # -------------------------
    # Gripper low-level helpers
    # -------------------------

    def _gripper(self, position, effort=None):
        """
        Thread-safe gripper control.
        """
        if effort is None:
            effort = self.GRIPPER_EFFORT

        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().warn(
                f"Gripper action server not ready: {self.GRIPPER_ACTION}"
            )
            return False

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(effort)

        send_future = self.gripper_client.send_goal_async(goal)
        
        # Wait for goal to be accepted (thread-safe)
        if not self._wait_for_future(send_future, timeout_sec=5.0):
            self.node.get_logger().warn("Gripper goal send timeout")
            return False

        goal_handle = send_future.result()
        if goal_handle is None:
            self.node.get_logger().warn("Gripper goal handle is None.")
            return False
        if not goal_handle.accepted:
            self.node.get_logger().warn("Gripper goal was rejected by server.")
            return False

        result_future = goal_handle.get_result_async()
        
        # Wait for result (thread-safe)
        if not self._wait_for_future(result_future, timeout_sec=5.0):
            self.node.get_logger().warn("Gripper result timeout")
            return False

        time.sleep(0.2)
        return True

    def gripper_open(self):
        return self._gripper(self.GRIPPER_OPEN_POS)

    def gripper_close(self):
        return self._gripper(self.GRIPPER_CLOSE_POS)

    # -------------------------
    # High-level sequences
    # -------------------------

    def grab(self):
        log = self.node.get_logger()
        log.debug("Arm grab sequence start (robust).")

        # 1) Open gripper
        self.gripper_open()

        # 2) Pre-grasp
        self._arm(0.0, -0.6, 0.3, 0.9, sec=1.5)

        # 3) Approach grasp
        self._arm(0.0, -0.8, 0.5, 1.0, sec=1.5)

        # 4) Close gripper
        self.gripper_close()

        # 5) Lift
        self._arm(0.0, -0.4, 0.2, 0.8, sec=1.5)

        log.debug("Arm grab sequence done.")

    def release(self):
        log = self.node.get_logger()
        log.debug("Arm release sequence start (robust).")

        # 1) Lower slightly
        self._arm(0.0, -0.7, 0.4, 0.9, sec=1.5)

        # 2) Open gripper
        self.gripper_open()

        # 3) Retract
        self._arm(0.0, -0.3, 0.1, 0.6, sec=1.5)

        log.debug("Arm release sequence done.")
