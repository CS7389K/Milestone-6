#!/usr/bin/env python3
"""
Launch file for Milestone 6 Part 1: Visual Servoing for Bottle Tracking

Part 1 Objective:
    Implement visual servoing that continuously tracks a bottle by rotating the robot
    to keep the bottle centered in the camera frame. The robot should lock onto the
    bottle and smoothly follow it as it moves.

Launches:
- TeleopPublisher Node: Centralized teleop service for base control
- YOLO Publisher Node: Captures camera frames and publishes bottle detections
- TeleopBase Node: Visual servoing controller (centering only, no approach)

Usage:
    ros2 launch milestone6 part1.launch.py

Optional Parameters:
    # Hardware Control
    include_hardware:=<bool>       Include hardware.launch.py (default: true)
    
    # YOLO Configuration
    yolo_model:=<path>             Path to YOLO model file (default: yolo11n.pt)
    image_width:=<int>             Camera image width in pixels (default: 1280)
    image_height:=<int>            Camera image height in pixels (default: 720)
    display:=<bool>                Display YOLO detections window (default: true)
    camera_backend:=<str>          Camera backend: gstreamer or opencv (default: gstreamer)
    camera_device:=<int>           Camera device ID (default: 1)
    gstreamer_pipeline:=<str>      Custom GStreamer pipeline (default: '')
    
    # Visual Servoing Parameters
    tracking_classes:=<str>        Comma-separated COCO class IDs (default: '39' for bottle)
    move_wheels:=<bool>            Enable robot movement (default: true)
    center_tolerance:=<int>        Centering tolerance in pixels (default: 30)
    turn_speed:=<float>            Angular velocity for turning rad/s (default: 0.5)
    detection_timeout:=<float>     Stop if no detection for N seconds (default: 0.5)

Examples:
    # Use different camera
    ros2 launch milestone6 part1.launch.py camera_device:=0
    
    # Track cups instead of bottles
    ros2 launch milestone6 part1.launch.py tracking_classes:='41'
    
    # Faster turning speed
    ros2 launch milestone6 part1.launch.py turn_speed:=1.0
    
    # Disable hardware launch (if running separately)
    ros2 launch milestone6 part1.launch.py include_hardware:=false

Demonstration:
    Move the bottle slowly left and right in front of the robot. The robot should
    smoothly rotate to keep the bottle centered in the camera frame.
"""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description with YOLO publisher, TeleopPublisher, and TeleopBase nodes."""
    # Declare launch arguments
    launch_args = [
        # Hardware control
        DeclareLaunchArgument('include_hardware', default_value='true',
                              description='Include hardware.launch.py'),

        # YOLO configuration
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

        # TeleopBase configuration
        DeclareLaunchArgument('tracking_classes', default_value='39',
                              description='Comma-separated COCO class IDs to track (39=bottle)'),
        DeclareLaunchArgument('move_wheels', default_value='true',
                              description='Enable robot movement'),
        DeclareLaunchArgument('center_tolerance', default_value='30',
                              description='Centering tolerance in pixels'),
        DeclareLaunchArgument('turn_speed', default_value='0.5',
                              description='Angular velocity in rad/s'),
        DeclareLaunchArgument('detection_timeout', default_value='0.5',
                              description='Stop if no detection for this many seconds'),
    ]
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
    # Teleop Publisher Node
    teleop_publisher_node = Node(
        package='milestone6',
        executable='teleop_publisher',
        name='teleop_publisher',
        output='screen',
    )
    # TeleopBase Node (Part 1: Visual Servoing - Centering Only)
    base_node = Node(
        package='milestone6',
        executable='base',
        name='teleop_base',
        output='screen',
        parameters=[{
            'tracking_classes': LaunchConfiguration('tracking_classes'),
            'image_width': LaunchConfiguration('image_width'),
            'move_wheels': LaunchConfiguration('move_wheels'),
            'center_tolerance': LaunchConfiguration('center_tolerance'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'detection_timeout': LaunchConfiguration('detection_timeout'),
        }]
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

    return LaunchDescription([
        # Launch arguments
        *launch_args,
        # Hardware (conditional)
        hardware_launch,
        # Nodes
        teleop_publisher_node,
        yolo_publisher_node,
        base_node,
    ])
