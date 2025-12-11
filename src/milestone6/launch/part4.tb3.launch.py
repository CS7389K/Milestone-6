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
    launch_args = [
        DeclareLaunchArgument('include_hardware', default_value='true',
                              description='Include hardware.launch.py (only use when running on TurtleBot)'),
        DeclareLaunchArgument('yolo_model', default_value='yolo11n.pt',
                              description='Path to YOLO model file'),
        DeclareLaunchArgument('image_width', default_value='1280',
                              description='Camera image width in pixels'),
        DeclareLaunchArgument('image_height', default_value='720',
                              description='Camera image height in pixels'),
        DeclareLaunchArgument('display', default_value='true',
                              description='Display YOLO detection window'),
        DeclareLaunchArgument('camera_backend', default_value='gstreamer',
                              description='Camera backend (gstreamer, opencv)'),
        DeclareLaunchArgument('camera_device', default_value='1',
                              description='Camera device ID'),
        DeclareLaunchArgument('gstreamer_pipeline', default_value='',
                              description='Custom GStreamer pipeline'),
        DeclareLaunchArgument('tracking_classes', default_value='39,73,64',
                              description='Comma-separated COCO class IDs (39=bottle, 73=bear, 64=mouse)'),
        # LLM action execution parameters
        DeclareLaunchArgument('turn_duration', default_value='1.0',
                              description='Duration for turn commands in seconds'),
        DeclareLaunchArgument('forward_duration', default_value='0.5',
                              description='Duration for forward commands in seconds'),
        DeclareLaunchArgument('scan_speed', default_value='0.5',
                              description='Rotation speed for scan command in rad/s'),
        DeclareLaunchArgument('forward_speed_llm', default_value='0.2',
                              description='Linear velocity for LLM forward commands in m/s'),
        # Visual servoing parameters
        DeclareLaunchArgument('center_tolerance', default_value='30',
                              description='Centering tolerance in pixels'),
        DeclareLaunchArgument('target_bbox_width', default_value='365',
                              description='Ideal bounding box width for grabbing in pixels'),
        DeclareLaunchArgument('forward_speed', default_value='0.05',
                              description='Linear velocity in m/s for visual servoing'),
        DeclareLaunchArgument('turn_speed', default_value='0.25',
                              description='Angular velocity in rad/s for visual servoing'),
        DeclareLaunchArgument('transport_distance', default_value='0.10',
                              description='Distance to transport object in meters'),
    ]

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
            'camera_backend': LaunchConfiguration('camera_backend'),
            'camera_device': LaunchConfiguration('camera_device'),
            'gstreamer_pipeline': LaunchConfiguration('gstreamer_pipeline'),
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
            'forward_speed_llm': LaunchConfiguration('forward_speed_llm'),
            'turn_duration': LaunchConfiguration('turn_duration'),
            'forward_duration': LaunchConfiguration('forward_duration'),
            'scan_speed': LaunchConfiguration('scan_speed'),
            'prompt_delay': LaunchConfiguration('prompt_delay'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'speed': LaunchConfiguration('speed'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'tracking_classes': LaunchConfiguration('tracking_classes'),
            'bbox_tolerance': LaunchConfiguration('bbox_tolerance'),
            'center_tolerance': LaunchConfiguration('center_tolerance'),
            'target_bbox_width': LaunchConfiguration('target_bbox_width'),
            'detection_timeout': LaunchConfiguration('detection_timeout'),
            'transport_distance': LaunchConfiguration('transport_distance'),
            'grab_joint2': LaunchConfiguration('grab_joint2'),
            'grab_joint3': LaunchConfiguration('grab_joint3'),
            'grab_joint4': LaunchConfiguration('grab_joint4'),
            'grab_vertical_adjust': LaunchConfiguration('grab_vertical_adjust'),
            'grasp_extension': LaunchConfiguration('grasp_extension'),
            'lift_joint2': LaunchConfiguration('lift_joint2'),
            'lift_joint3': LaunchConfiguration('lift_joint3'),
            'lift_joint4': LaunchConfiguration('lift_joint4'),
            'lower_joint1': LaunchConfiguration('lower_joint1'),
            'lower_joint2': LaunchConfiguration('lower_joint2'),
            'lower_joint3': LaunchConfiguration('lower_joint3'),
            'lower_joint4': LaunchConfiguration('lower_joint4'),
            'home_joint1': LaunchConfiguration('home_joint1'),
            'home_joint2': LaunchConfiguration('home_joint2'),
            'home_joint3': LaunchConfiguration('home_joint3'),
            'home_joint4': LaunchConfiguration('home_joint4'),
            'gripper_open': LaunchConfiguration('gripper_open'),
            'gripper_close': LaunchConfiguration('gripper_close'),
        }]
    )

    return LaunchDescription([
        # Launch arguments
        *launch_args,
        # Nodes
        hardware_launch,
        yolo_publisher_node,
        mission_node,
    ])
