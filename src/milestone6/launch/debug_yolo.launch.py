#!/usr/bin/env python3
"""
Launch file for YOLO Debug Mode

Launches both YOLO Publisher and Debug Subscriber with DEBUG logging enabled.
Perfect for testing and debugging YOLO detection pipeline.

Usage:
    ros2 launch milestone6 yolo_debug.launch.py

Optional Parameters:
    yolo_model:=<path>     Path to YOLO model file (default: yolo11n.pt)
    image_width:=<int>     Camera image width (default: 500)
    image_height:=<int>    Camera image height (default: 320)
    display:=<bool>        Display YOLO detections window (default: true)

Example:
    ros2 launch milestone6 yolo_debug.launch.py display:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with YOLO publisher and debug subscriber."""
    
    # Declare launch arguments
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolo11n.pt',
        description='Path to YOLO model file'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='500',
        description='Camera image width'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='320',
        description='Camera image height'
    )
    
    display_arg = DeclareLaunchArgument(
        'display',
        default_value='true',
        description='Display YOLO detections window'
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
        }],
        arguments=['--ros-args', '--log-level', 'DEBUG']
    )
    
    # YOLO Debug Subscriber Node
    yolo_debug_subscriber_node = Node(
        package='milestone6',
        executable='yolo_debug_subscriber',
        name='yolo_debug_subscriber',
        output='screen',
        arguments=['--ros-args', '--log-level', 'DEBUG']
    )
    
    return LaunchDescription([
        yolo_model_arg,
        image_width_arg,
        image_height_arg,
        display_arg,
        yolo_publisher_node,
        yolo_debug_subscriber_node,
    ])
