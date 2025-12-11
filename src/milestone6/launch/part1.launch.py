#!/usr/bin/env python3
"""
Launch file for Milestone 6 Part 1: Visual Servoing for Bottle Tracking

Objective:
    Implement visual servoing that continuously tracks a bottle by rotating the robot
    to keep the bottle centered in the camera frame. The robot should lock onto the
    bottle and smoothly follow it as it moves.

Launches:
- TeleopPublisher Node: Centralized teleop service for base control
- YOLO Publisher Node: Captures camera frames and publishes bottle detections
- Part1 Node: Visual servoing controller (centering only, no approach)

Usage:
    ros2 launch milestone6 part1.launch.py

Optional Parameters:
    include_hardware:=<bool>       Include hardware.launch.py (default: true)
    yolo_model:=<path>             Path to YOLO model file (default: yolo11n.pt)
    image_width:=<int>             Camera image width in pixels (default: 1280)
    image_height:=<int>            Camera image height in pixels (default: 720)
    display:=<bool>                Display YOLO detections window (default: true)
    camera_backend:=<str>          Camera backend: gstreamer or opencv (default: gstreamer)
    camera_device:=<int>           Camera device ID (default: 1)
    gstreamer_pipeline:=<str>      Custom GStreamer pipeline (default: '')
    tracking_classes:=<str>        Comma-separated COCO class IDs (default: '39' for bottle)
    speed:=<float>                 Linear movement speed in m/s (default: 0.05)
    bbox_tolerance:=<int>          Bounding box width tolerance in pixels (default: 20)
    center_tolerance:=<int>        Centering tolerance in pixels (default: 30)
    target_bbox_width:=<int>       Target bounding box width for approach in pixels (default: 365)
    turn_speed:=<float>            Angular velocity for turning rad/s (default: 0.25)
    detection_timeout:=<float>     Stop if no detection for N seconds (default: 1.0)
"""


from launch import LaunchDescription
from launch_ros.actions import Node

from .util import HARDWARE_LAUNCH, LaunchArg

LAUNCH_ARGS = {
    'include_hardware': ('true', 'Include hardware.launch.py'),
    'yolo_model': ('yolo11n.pt', 'Path to YOLO model file'),
    'image_width': ('1280', 'Camera image width in pixels'),
    'image_height': ('720', 'Camera image height in pixels'),
    'display': ('true', 'Display YOLO detection window'),
    'camera_backend': ('gstreamer', 'Camera backend (gstreamer, opencv)'),
    'camera_device': ('1', 'Camera device ID'),
    'gstreamer_pipeline': ('', 'Custom GStreamer pipeline'),
    'tracking_classes': ('39', 'Comma-separated COCO class IDs to track (39=bottle)'),
    'speed': ('0.05', 'Linear movement speed in m/s'),
    'bbox_tolerance': ('20', 'Bounding box width tolerance in pixels'),
    'center_tolerance': ('30', 'Centering tolerance in pixels'),
    'target_bbox_width': ('365', 'Target bounding box width for approach in pixels'),
    'turn_speed': ('0.25', 'Angular velocity in rad/s'),
    'detection_timeout': ('1.0', 'Stop if no detection for this many seconds'),
}


def generate_launch_description():
    """Generate launch description with YOLO publisher, TeleopPublisher, and TeleopBase nodes."""
    # Generate launch arguments
    launch_args = LaunchArg.generate_launch_arguments(LAUNCH_ARGS)

    # YOLO Publisher Node
    yolo_publisher_node = Node(
        package='milestone6',
        executable='yolo_publisher',
        name='yolo_publisher',
        output='screen',
        parameters=[
            LaunchArg.generate_launch_configs([
                'yolo_model',
                'image_width',
                'image_height',
                'display',
                'camera_backend',
                'camera_device',
                'gstreamer_pipeline',
            ])
        ]
    )

    # Teleop Publisher Node
    teleop_publisher_node = Node(
        package='milestone6',
        executable='teleop_publisher',
        name='teleop_publisher',
        output='screen',
    )

    # Part 1 Node
    part1_node = Node(
        package='milestone6',
        executable='part1',
        name='part1',
        output='screen',
        parameters=[
            LaunchArg.generate_launch_configs([
                'image_width',
                'image_height',
                'speed',
                'turn_speed',
                'tracking_classes',
                'bbox_tolerance',
                'center_tolerance',
                'target_bbox_width',
                'detection_timeout',
            ])
        ]
    )

    return LaunchDescription([
        *launch_args,
        HARDWARE_LAUNCH,
        teleop_publisher_node,
        yolo_publisher_node,
        part1_node,
    ])
