#!/usr/bin/env python3
"""
Launch file for Milestone 6 Part 1: YOLO Publisher + ML6 Server

Launches:
- YOLO Publisher Node: Captures camera frames and publishes object detections
- ML6 Server Node: Subscribes to detections and controls robot movement

Usage:
    ros2 launch milestone6 part1.launch.py

Optional Parameters:
    yolo_model:=<path>     Path to YOLO model file (default: yolo11n.pt)
    image_width:=<int>     Camera image width (default: 500)
    image_height:=<int>    Camera image height (default: 320)
    display:=<bool>        Display YOLO detections window (default: true)
    track_class:=<int>     COCO class ID to track (default: 39 for bottle)
    move_wheels:=<bool>    Enable robot movement (default: true)
    tolerance:=<int>       Centering tolerance in pixels (default: 50)
    min_bbox_width:=<int>  Min bbox width to stop (default: 150)
    forward_speed:=<float> Linear velocity (default: 0.15)
    turn_speed:=<float>    Angular velocity (default: 1.5)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with YOLO publisher and ML6 server nodes."""
    
    # YOLO Publisher Node
    yolo_publisher_node = Node(
        package='milestone6',
        executable='yolo_publisher',
        name='yolo_publisher',
        output='screen',
        parameters=[{
            'yolo_model': 'yolo11n.pt',
            'image_width': 500,
            'image_height': 320,
            'display': True,
        }]
    )
    
    # ML6 Server Node
    server_node = Node(
        package='milestone6',
        executable='server',
        name='m6_server',
        output='screen',
        parameters=[{
            'move_wheels': True,
            'image_width': 500,
            'image_height': 320,
            'tolerance': 50,
            'min_bbox_width': 150,
            'forward_speed': 0.15,
            'turn_speed': 1.5,
            'track_class': 39,  # bottle
        }]
    )
    
    return LaunchDescription([
        yolo_publisher_node,
        server_node,
    ])
