import rclpy
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from control_msgs.action import GripperCommand, FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Duration

from .constants import (
    SERVO_START_SRV,
    SERVO_STOP_SRV,
    BASE_TWIST_TOPIC,
    ARM_JOINT_TOPIC,
    GRIPPER_ACTION,
    ROS_QUEUE_SIZE,
    BASE_LINEAR_VEL_MAX,
    BASE_LINEAR_VEL_STEP,
    BASE_ANGULAR_VEL_MAX,
    BASE_ANGULAR_VEL_STEP,
    JOINT_NAMES,
    TRAJ_TIME_S,
    POSES,
    GRIPPER_OPEN_POSITION,
    GRIPPER_CLOSE_POSITION,
)


class TeleopPublisher:
    """
    Publisher for TurtleBot3 + OpenManipulatorX teleoperation.
    
    Publishes velocity commands, arm trajectories, and gripper commands.
    Follows the publisher pattern similar to YOLOPublisher.
    
    Based on turtlebot3_manipulation_teleop:
    - https://github.com/ROBOTIS-GIT/turtlebot3_manipulation/blob/humble/turtlebot3_manipulation_teleop/
    """

    def __init__(self, node):
        """
        Initialize the teleop publisher.
        
        Args:
            node: The parent ROS2 Node instance
            joint_state_subscriber: Optional JointStateSubscriber for arm control
        """
        self._node = node
        
        # Create service clients for MoveIt servo
        self.servo_start_client = node.create_client(Trigger, SERVO_START_SRV)
        self.servo_stop_client = node.create_client(Trigger, SERVO_STOP_SRV)
        
        # Create publishers
        self.base_twist_pub = node.create_publisher(Twist, BASE_TWIST_TOPIC, ROS_QUEUE_SIZE)
        
        # Create action clients
        self.arm_traj_ac = ActionClient(node, FollowJointTrajectory, ARM_JOINT_TOPIC)
        self.gripper_client = ActionClient(node, GripperCommand, GRIPPER_ACTION)

        # Create timer for publishing velocity commands
        self.pub_timer = node.create_timer(0.01, self.publish_loop)
        
        # Current velocity command
        self.cmd_vel = Twist()

        # MoveIt servo interface (optional, not needed for autonomous navigation)
        # Uncomment if you need advanced servo control:
        # self.connect_moveit_servo()
        # self.start_moveit_servo()

    def connect_moveit_servo(self):
        """Connect to MoveIt servo services."""
        for i in range(10):
            if self.servo_start_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('SUCCESS TO CONNECT SERVO START SERVER')
                break
            self.get_logger().warn('WAIT TO CONNECT SERVO START SERVER')
            if i == 9:
                self.get_logger().error(
                    "fail to connect moveit_servo. please launch 'servo.launch' from the MoveIt config package."
                )

        for i in range(10):
            if self.servo_stop_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('SUCCESS TO CONNECT SERVO STOP SERVER')
                break
            self.get_logger().warn('WAIT TO CONNECT SERVO STOP SERVER')
            if i == 9:
                self.get_logger().error(
                    "fail to connect moveit_servo. please launch 'servo.launch' from the MoveIt config package."
                )

    def start_moveit_servo(self):
        """Start MoveIt servo interface."""
        self.get_logger().info("call 'moveit_servo' start srv.")
        if not self.servo_start_client.service_is_ready():
            self.get_logger().warn("start_servo service not ready; continuing without moveit_servo.")
            return
        future = self.servo_start_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=1.0)
        if future.done() and future.result():
            self.get_logger().info("SUCCESS to start 'moveit_servo'")
        else:
            self.get_logger().error("FAIL to start 'moveit_servo', executing without 'moveit_servo'")

    def stop_moveit_servo(self):
        """Shutdown MoveIt servo interface."""
        self.get_logger().info("call 'moveit_servo' END srv.")
        if not self.servo_stop_client.service_is_ready():
            return
        future = self.servo_stop_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=1.0)

    def publish_loop(self):
        """Publish velocity commands periodically."""
        self.base_twist_pub.publish(self.cmd_vel)

    def set_velocity(self, linear_x: float = 0.0, angular_z: float = 0.0):
        """
        Set velocity command directly.
        
        Args:
            linear_x: Linear velocity in x direction (m/s)
            angular_z: Angular velocity around z axis (rad/s)
        """
        self.cmd_vel.linear.x = float(linear_x)
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = float(angular_z)

    def inc_linear(self):
        """Increase linear velocity."""
        self.cmd_vel.linear.x = min(self.cmd_vel.linear.x + BASE_LINEAR_VEL_STEP, BASE_LINEAR_VEL_MAX)
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self._node.get_logger().info(f'Linear velocity: {self.cmd_vel.linear.x:.3f}')

    def dec_linear(self):
        """Decrease linear velocity."""
        self.cmd_vel.linear.x = max(self.cmd_vel.linear.x - BASE_LINEAR_VEL_STEP, -BASE_LINEAR_VEL_MAX)
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self._node.get_logger().info(f'Linear velocity: {self.cmd_vel.linear.x:.3f}')

    def inc_ang(self):
        """Increase angular velocity."""
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = min(self.cmd_vel.angular.z + BASE_ANGULAR_VEL_STEP, BASE_ANGULAR_VEL_MAX)
        self._node.get_logger().info(f'Angular velocity: {self.cmd_vel.angular.z:.3f}')

    def dec_ang(self):
        """Decrease angular velocity."""
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = max(self.cmd_vel.angular.z - BASE_ANGULAR_VEL_STEP, -BASE_ANGULAR_VEL_MAX)
        self._node.get_logger().info(f'Angular velocity: {self.cmd_vel.angular.z:.3f}')

    def stop(self):
        """Stop the base."""
        self.cmd_vel = Twist()

    def send_gripper_goal(self, position: float):
        """
        Send gripper command.
        
        Args:
            position (float): Gripper position in meters
                             positive to open (~0.025), negative to close (~-0.015)
        """
        if not self.gripper_client.server_is_ready():
            self._node.get_logger().warn('Gripper action server not ready.')
            return

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = -1.0

        self.gripper_client.send_goal_async(goal)

    def gripper_open(self):
        """Open the gripper."""
        self.send_gripper_goal(GRIPPER_OPEN_POSITION)

    def gripper_close(self):
        """Close the gripper."""
        self.send_gripper_goal(GRIPPER_CLOSE_POSITION)

    def send_arm_trajectory(self, target_positions: dict):
        """
        Send arm trajectory goal.
        
        Args:
            target_positions: Dictionary mapping joint names to target positions (radians)
        """
        if not self.arm_traj_ac.server_is_ready():
            self._node.get_logger().warn('FollowJointTrajectory action server not ready.')
            return

        jt = JointTrajectory()
        jt.joint_names = list(JOINT_NAMES)

        pt = JointTrajectoryPoint()
        pt.positions = [target_positions.get(j, 0.0) for j in JOINT_NAMES]
        pt.time_from_start = Duration(
            sec=int(TRAJ_TIME_S),
            nanosec=int((TRAJ_TIME_S % 1.0) * 1e9)
        )
        jt.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        self.arm_traj_ac.send_goal_async(goal)

    def shutdown(self):
        """Shutdown the teleop publisher."""
        self._node.get_logger().info('Shutting down teleop publisher...')
        self.stop()
        # self.stop_moveit_servo()  # Only needed if servo was started
        self.pub_timer.cancel()
        self.pub_timer = None

    def get_logger(self):
        """Get the node's logger."""
        return self._node.get_logger()


from rclpy.node import Node


class TeleopPublisherNode(Node):
    """
    Standalone TeleopPublisher Node.
    
    Publishes base velocity commands, arm trajectories, and gripper commands.
    Can be used as a centralized teleop service for other nodes.
    """
    def __init__(self):
        super().__init__('teleop_publisher')
        self.get_logger().info("Starting TeleopPublisher Node...")
        
        # Initialize the publisher (using self as the node)
        self.publisher = TeleopPublisher(self)
        
        self.get_logger().info("TeleopPublisher Node ready.")
    
    def shutdown(self):
        """Shutdown the node."""
        self.get_logger().info('Shutting down TeleopPublisher Node...')
        self.publisher.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TeleopPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down TeleopPublisher Node...")
    except Exception as e:
        print(f"Error in TeleopPublisher Node: {e}")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()