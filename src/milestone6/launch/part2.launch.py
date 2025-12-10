#!/usr/bin/env python3
"""
Launch file for Milestone 6 Part 2: Complete Object Grab and Transport Mission

This launch file starts all required nodes for the Part 2 mission:
1. YOLO Publisher - Object detection using camera
2. Part 2 Mission Node - Complete autonomous grab and transport sequence

Mission Flow:
    IDLE -> CENTERING -> APPROACHING -> GRABBING -> TRANSPORTING -> RELEASING -> DONE

Requirements:
    - Hardware controllers must be running first:
      ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
    
    - Or simulated environment:
      ros2 launch turtlebot3_manipulation_bringup controllers.launch.py

Usage:
    # Start hardware first (on TurtleBot):
    ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
    
    # Then start Part 2 mission (on remote PC or TurtleBot):
    ros2 launch milestone6 part2_complete.launch.py

Optional Parameters:
    yolo_model:=<path>          YOLO model file (default: yolo11n.pt)
    image_width:=<int>          Camera width (default: 500)
    image_height:=<int>         Camera height (default: 320)
    display:=<bool>             Show YOLO window (default: true)
    tracking_classes:=<str>     Comma-separated COCO class IDs (default: '39' for bottle)
    center_tolerance:=<int>     Centering tolerance px (default: 30)
    target_bbox_width:=<int>    Ideal bbox width (default: 180)
    forward_speed:=<float>      Linear velocity m/s (default: 0.15)
    turn_speed:=<float>         Angular velocity rad/s (default: 1.0)
    transport_distance:=<float> Transport distance meters (default: 1.0)

Examples:
    # Track cups instead of bottles
    ros2 launch milestone6 part2_complete.launch.py tracking_classes:='41'
    
    # Track both bottles and cups
    ros2 launch milestone6 part2_complete.launch.py tracking_classes:='39,41'
    
    # Disable YOLO display window
    ros2 launch milestone6 part2_complete.launch.py display:=false
    
    # Transport 0.5 meters instead of 1 meter
    ros2 launch milestone6 part2_complete.launch.py transport_distance:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Part 2 mission."""

    # Declare launch arguments
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
        default_value='39',
        description='Comma-separated COCO class IDs to track (e.g., "39,41" for bottle and cup)'
    )

    center_tolerance_arg = DeclareLaunchArgument(
        'center_tolerance',
        default_value='30',
        description='Centering tolerance in pixels'
    )

    target_bbox_width_arg = DeclareLaunchArgument(
        'target_bbox_width',
        default_value='180',
        description='Ideal bounding box width for grabbing in pixels'
    )

    forward_speed_arg = DeclareLaunchArgument(
        'forward_speed',
        default_value='0.15',
        description='Linear velocity in m/s'
    )

    turn_speed_arg = DeclareLaunchArgument(
        'turn_speed',
        default_value='1.0',
        description='Angular velocity in rad/s'
    )

    transport_distance_arg = DeclareLaunchArgument(
        'transport_distance',
        default_value='1.0',
        description='Distance to transport object in meters (3.2ft = 0.975m)'
    )

    detection_timeout_arg = DeclareLaunchArgument(
        'detection_timeout',
        default_value='1',
        description='Detection timeout in seconds'
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

    # Part 2 Mission Node
    mission_node = Node(
        package='milestone6',
        executable='part2',
        name='part2',
        output='screen',
        parameters=[{
            'tracking_classes': LaunchConfiguration('tracking_classes'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'bbox_tolerance': 20,
            'center_tolerance': LaunchConfiguration('center_tolerance'),
            'target_bbox_width': LaunchConfiguration('target_bbox_width'),
            'forward_speed': LaunchConfiguration('forward_speed'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'detection_timeout': LaunchConfiguration('detection_timeout'),
            'transport_distance': LaunchConfiguration('transport_distance'),
        }]
    )

    return LaunchDescription([
        # Launch arguments
        yolo_model_arg,
        image_width_arg,
        image_height_arg,
        display_arg,
        tracking_classes_arg,
        center_tolerance_arg,
        target_bbox_width_arg,
        forward_speed_arg,
        turn_speed_arg,
        transport_distance_arg,
        detection_timeout_arg,
        # Nodes
        yolo_publisher_node,
        mission_node
    ])
