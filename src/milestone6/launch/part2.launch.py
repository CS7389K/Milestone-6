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
    target_class:=<int>         COCO class ID (default: 39 for bottle)
    center_tolerance:=<int>     Centering tolerance px (default: 30)
    target_bbox_width:=<int>    Ideal bbox width (default: 180)
    forward_speed:=<float>      Linear velocity m/s (default: 0.15)
    turn_speed:=<float>         Angular velocity rad/s (default: 1.0)
    transport_distance:=<float> Transport distance meters (default: 1.0)

Examples:
    # Track cups instead of bottles
    ros2 launch milestone6 part2_complete.launch.py target_class:=41
    
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
        default_value='500',
        description='Camera image width in pixels'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='320',
        description='Camera image height in pixels'
    )

    display_arg = DeclareLaunchArgument(
        'display',
        default_value='true',
        description='Display YOLO detection window'
    )

    target_class_arg = DeclareLaunchArgument(
        'target_class',
        default_value='39',
        description='COCO class ID to track (39=bottle, 41=cup)'
    )

    center_tolerance_arg = DeclareLaunchArgument(
        'center_tolerance',
        default_value='30',
        description='Centering tolerance in pixels'
    )

    target_bbox_arg = DeclareLaunchArgument(
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
        default_value='1.0',
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
            'target_class': LaunchConfiguration('target_class'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'center_tolerance': LaunchConfiguration('center_tolerance'),
            'target_bbox_width': LaunchConfiguration('target_bbox_arg'),
            'bbox_tolerance': 20,
            'forward_speed': LaunchConfiguration('forward_speed'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'transport_distance': LaunchConfiguration('transport_distance'),
            'detection_timeout': LaunchConfiguration('detection_timeout'),
        }]
    )

    return LaunchDescription([
        # Launch arguments
        yolo_model_arg,
        image_width_arg,
        image_height_arg,
        display_arg,
        target_class_arg,
        center_tolerance_arg,
        target_bbox_arg,
        forward_speed_arg,
        turn_speed_arg,
        transport_distance_arg,
        detection_timeout_arg,
        # Nodes
        yolo_publisher_node,
        mission_node,
    ])
