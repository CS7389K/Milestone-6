# Teleop constants for TurtleBot3 + OpenManipulatorX control

# Service names
SERVO_START_SRV = '/servo_server/start_servo'
SERVO_STOP_SRV = '/servo_server/stop_servo'

# Topic names
BASE_TWIST_TOPIC = '/cmd_vel'
ARM_JOINT_TOPIC = '/arm_controller/follow_joint_trajectory'
GRIPPER_ACTION = '/gripper_controller/gripper_cmd'

# ROS queue size
ROS_QUEUE_SIZE = 10

# Base movement parameters
BASE_LINEAR_VEL_MAX = 0.26
BASE_LINEAR_VEL_STEP = 0.01
BASE_ANGULAR_VEL_MAX = 1.8
BASE_ANGULAR_VEL_STEP = 0.1

# Arm control parameters
JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4")
JOINT_STEP = 0.05  # radians per key press
TRAJ_TIME_S = 0.6  # seconds to reach each nudge

# Predefined arm poses (ROBOTIS-tested safe positions)
# Source: turtlebot3_manipulation opencr.cpp
POSES = {
    # Home pose - safe retracted position (tested by ROBOTIS)
    "home": {"joint1": 0.0, "joint2": -1.05, "joint3": 0.35, "joint4": 0.70},
    
    # Init pose - forward reach for manipulation (~22cm forward, ~18cm high)
    # Expected bbox width at this distance: ~180px for 7cm bottle
    "init": {"joint1": 0.0, "joint2": -1.57, "joint3": 1.37, "joint4": 0.26},
    
    # Zero pose - all joints straight
    "zero": {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0, "joint4": 0.0},
}

# Gripper positions
GRIPPER_OPEN_POSITION = 0.025
GRIPPER_CLOSE_POSITION = -0.015
