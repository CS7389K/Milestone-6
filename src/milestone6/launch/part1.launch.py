#!/usr/bin/env python3
"""
Launch file for Milestone 6 Part 1: TeleopPublisher + YOLO Publisher + TeleopBase

Launches:
- TeleopPublisher Node: Centralized teleop service for base and arm control
- YOLO Publisher Node: Captures camera frames and publishes object detections
- TeleopBase Node: Subscribes to detections and controls robot base movement

Usage:
    ros2 launch milestone6 part1.launch.py

Optional Parameters:
    yolo_model:=<path>     Path to YOLO model file (default: yolo11n.pt)
    image_width:=<int>     Camera image width (default: 500)
    image_height:=<int>    Camera image height (default: 320)
    display:=<bool>        Display YOLO detections window (default: true)
    tracking_classes:=<str> Comma-separated COCO class IDs to track (default: '39' for bottle)
    move_wheels:=<bool>    Enable robot movement (default: true)
    center_tolerance:=<int> Centering tolerance in pixels (default: 30)
    min_bbox_width:=<int>  Min bbox width to stop (default: 150)
    forward_speed:=<float> Linear velocity (default: 0.15)
    turn_speed:=<float>    Angular velocity (default: 1.5)
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
    # Declare launch arguments for YOLO publisher
    yolo_args = [
        DeclareLaunchArgument('yolo_model', default_value='yolo11n.pt'),
        DeclareLaunchArgument('image_width', default_value='500'),
        DeclareLaunchArgument('image_height', default_value='320'),
        DeclareLaunchArgument('display', default_value='true'),
        DeclareLaunchArgument('camera_backend', default_value='gstreamer'),
        DeclareLaunchArgument('camera_device', default_value='1'),
        DeclareLaunchArgument('gstreamer_pipeline', default_value=''),
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
    # TeleopBase Node
    base_node = Node(
        package='milestone6',
        executable='base',
        name='teleop_base',
        output='screen',
        parameters=[{
            'tracking_classes': '39',  # bottle
            'image_width': 1280,
            'move_wheels': True,
            'bbox_tolerance': 20,
            'center_tolerance': 30,
            'target_bbox_width': 180,
            'forward_speed': 0.15,
            'turn_speed': 1.0,
            'detection_timeout': 0.5,
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
        *yolo_args,
        # Hardware (conditional)
        hardware_launch,
        # Nodes
        teleop_publisher_node,
        yolo_publisher_node,
        base_node,
    ])
