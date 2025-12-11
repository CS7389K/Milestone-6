#!/usr/bin/env python3
"""
Part 4 TurtleBot Launch File

This launch file runs on the TurtleBot and includes:
1. Hardware bringup (arm and base controllers)
2. YOLO Publisher (multi-object detection: bottle, bear, mouse)
3. Part 4 Mission Node (LLM-guided search and retrieval)

This node subscribes to /llm_action from a remote LLaMA publisher.
The LLaMA publisher should be run separately on a powerful PC using part4.jetson.launch.py

Usage:
    # On TurtleBot:
    ros2 launch milestone6 part4.tb3.launch.py
    
    # On Remote PC (separately):
    ros2 launch milestone6 part4.jetson.launch.py

Optional Parameters:
    include_hardware:=<bool>    Include hardware.launch.py (default: true)
    yolo_model:=<str>           Path to YOLO model (default: yolo11n.pt)
    image_width:=<int>          Camera width in pixels (default: 1280)
    image_height:=<int>         Camera height in pixels (default: 720)
    display:=<bool>             Display YOLO window (default: true)
    tracking_classes:=<str>     COCO class IDs to track (default: '39,73,64')
                                39=bottle, 73=teddy bear, 64=mouse

Examples:
    # Run without hardware (for testing)
    ros2 launch milestone6 part4.tb3.launch.py include_hardware:=false
    
    # Run with different tracking classes
    ros2 launch milestone6 part4.tb3.launch.py tracking_classes:='39,73'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Part 4 TurtleBot side."""

    # Declare launch arguments
    include_hardware_arg = DeclareLaunchArgument(
        'include_hardware',
        default_value='true',
        description='Include hardware.launch.py (only use when running on TurtleBot)'
    )

    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolo11n.pt',
        description='Path to YOLO model file'
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='1280',
        description='Camera image width in pixels'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='720',
        description='Camera image height in pixels'
    )

    display_arg = DeclareLaunchArgument(
        'display',
        default_value='true',
        description='Display YOLO detection window'
    )

    tracking_classes_arg = DeclareLaunchArgument(
        'tracking_classes',
        default_value='39,73,64',
        description='Comma-separated COCO class IDs (39=bottle, 73=bear, 64=mouse)'
    )

    # LLM action execution parameters
    turn_duration_arg = DeclareLaunchArgument(
        'turn_duration',
        default_value='1.0',
        description='Duration for turn commands in seconds'
    )

    forward_duration_arg = DeclareLaunchArgument(
        'forward_duration',
        default_value='0.5',
        description='Duration for forward commands in seconds'
    )

    scan_speed_arg = DeclareLaunchArgument(
        'scan_speed',
        default_value='0.5',
        description='Rotation speed for scan command in rad/s'
    )

    forward_speed_llm_arg = DeclareLaunchArgument(
        'forward_speed_llm',
        default_value='0.2',
        description='Linear velocity for LLM forward commands in m/s'
    )

    # Visual servoing parameters
    center_tolerance_arg = DeclareLaunchArgument(
        'center_tolerance',
        default_value='30',
        description='Centering tolerance in pixels'
    )

    target_bbox_width_arg = DeclareLaunchArgument(
        'target_bbox_width',
        default_value='365',
        description='Ideal bounding box width for grabbing in pixels'
    )

    forward_speed_arg = DeclareLaunchArgument(
        'forward_speed',
        default_value='0.05',
        description='Linear velocity in m/s for visual servoing'
    )

    turn_speed_arg = DeclareLaunchArgument(
        'turn_speed',
        default_value='0.25',
        description='Angular velocity in rad/s for visual servoing'
    )

    transport_distance_arg = DeclareLaunchArgument(
        'transport_distance',
        default_value='0.10',
        description='Distance to transport object in meters'
    )

    # Hardware Launch (conditional)
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('turtlebot3_manipulation_bringup'),
            '/launch/hardware.launch.py'
        ]),
        launch_arguments={
            'log_level': 'error'
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_hardware'))
    )

    # YOLO Publisher Node
    yolo_publisher_node = Node(
        package='milestone6',
        executable='yolo_publisher',
        name='yolo_publisher',
        output='screen',
        parameters=[{
            'yolo_model': LaunchConfiguration('yolo_model'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'display': LaunchConfiguration('display'),
        }]
    )

    # Part 4 Mission Node (LLM-guided)
    mission_node = Node(
        package='milestone6',
        executable='part4',
        name='part4',
        output='screen',
        parameters=[{
            # LLM action parameters
            'turn_duration': LaunchConfiguration('turn_duration'),
            'forward_duration': LaunchConfiguration('forward_duration'),
            'scan_speed': LaunchConfiguration('scan_speed'),
            'forward_speed_llm': LaunchConfiguration('forward_speed_llm'),
            # Visual servoing parameters
            'tracking_classes': LaunchConfiguration('tracking_classes'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'bbox_tolerance': 20,
            'center_tolerance': LaunchConfiguration('center_tolerance'),
            'target_bbox_width': LaunchConfiguration('target_bbox_width'),
            'forward_speed': LaunchConfiguration('forward_speed'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'detection_timeout': 1,
            'transport_distance': LaunchConfiguration('transport_distance'),
            # Arm grab positions
            'grab_joint2': 0.95,
            'grab_joint3': -0.65,
            'grab_joint4': 0.0,
            'grab_vertical_adjust': 0.2,
            'grasp_extension': 0.2,
            # Arm lift positions
            'lift_joint2': -0.5,
            'lift_joint3': 0.4,
            'lift_joint4': 0.6,
            # Arm lower positions
            'lower_joint1': 0.0,
            'lower_joint2': 0.6,
            'lower_joint3': -0.4,
            'lower_joint4': 0.6,
            # Arm home positions
            'home_joint1': 0.0,
            'home_joint2': -1.05,
            'home_joint3': 0.35,
            'home_joint4': 0.70,
            # Gripper positions
            'gripper_open': 0.025,
            'gripper_close': -0.015,
        }]
    )

    return LaunchDescription([
        # Launch arguments
        include_hardware_arg,
        yolo_model_arg,
        image_width_arg,
        image_height_arg,
        display_arg,
        tracking_classes_arg,
        turn_duration_arg,
        forward_duration_arg,
        scan_speed_arg,
        forward_speed_llm_arg,
        center_tolerance_arg,
        target_bbox_width_arg,
        forward_speed_arg,
        turn_speed_arg,
        transport_distance_arg,
        # Nodes
        hardware_launch,
        yolo_publisher_node,
        mission_node,
    ])
