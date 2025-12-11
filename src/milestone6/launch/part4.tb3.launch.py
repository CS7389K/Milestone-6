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
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from milestone6.util.launch import HARDWARE_LAUNCH, LaunchArg

LAUNCH_ARGS = {
    'include_hardware': ('true', 'Include hardware.launch.py'),
    'yolo_model': ('yolo11n.pt', 'Path to YOLO model file'),
    'image_width': ('1280', 'Camera image width in pixels'),
    'image_height': ('720', 'Camera image height in pixels'),
    'display': ('true', 'Display YOLO detection window'),
    'camera_backend': ('gstreamer', 'Camera backend (gstreamer, opencv)'),
    'camera_device': ('1', 'Camera device ID'),
    'gstreamer_pipeline': ('', 'Custom GStreamer pipeline'),
    'speed': ('0.05', 'Linear movement speed in m/s'),
    'turn_speed': ('0.25', 'Angular velocity in rad/s'),
    'tracking_classes': ('39', 'Comma-separated COCO class IDs to track (39=bottle)'),
    'bbox_tolerance': ('20', 'Bounding box width tolerance in pixels'),
    'center_tolerance': ('30', 'Centering tolerance in pixels'),
    'target_bbox_width': ('365', 'Target bounding box width for approach in pixels'),
    'detection_timeout': ('1.0', 'Stop if no detection for this many seconds'),
    'transport_distance': ('0.10', 'Distance to transport object in meters'),
    'grab_joint2': ('0.95', 'Grab position: forward reach (radians)'),
    'grab_joint3': ('-0.65', 'Grab position: extend elbow (radians)'),
    'grab_joint4': ('0.0', 'Grab position: level gripper (radians)'),
    'grab_vertical_adjust': ('0.2', 'Extra reach for low objects (radians)'),
    'grasp_extension': ('0.2', 'Extra extension to grasp (radians)'),
    'lift_joint2': ('-0.5', 'Lift position: lift up (radians)'),
    'lift_joint3': ('0.4', 'Lift position: retract (radians)'),
    'lift_joint4': ('0.6', 'Lift position: adjust wrist (radians)'),
    'lower_joint1': ('0.0', 'Lower position: center (radians)'),
    'lower_joint2': ('0.6', 'Lower position: down (radians)'),
    'lower_joint3': ('-0.4', 'Lower position: extend (radians)'),
    'lower_joint4': ('0.6', 'Lower position: level (radians)'),
    'home_joint1': ('0.0', 'Home position: joint1 (radians)'),
    'home_joint2': ('-1.05', 'Home position: joint2 (radians)'),
    'home_joint3': ('0.35', 'Home position: joint3 (radians)'),
    'home_joint4': ('0.70', 'Home position: joint4 (radians)'),
    'gripper_open': ('0.025', 'Gripper open position'),
    'gripper_close': ('-0.015', 'Gripper close position'),
    'turn_duration': ('1.0', 'Duration for turn commands in seconds'),
    'forward_duration': ('0.5', 'Duration for forward commands in seconds'),
    'scan_speed': ('0.5', 'Rotation speed for scan command in rad/s'),
    'forward_speed_llm': ('0.2', 'Linear velocity for LLM forward commands in m/s'),
}


def generate_launch_description():
    """Generate launch description for Part 4 TurtleBot side."""
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

    # Part 4 Mission Node (LLM-guided)
    mission_node = Node(
        package='milestone6',
        executable='part4',
        name='part4',
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
                'transport_distance',
                'grab_joint2',
                'grab_joint3',
                'grab_joint4',
                'grab_vertical_adjust',
                'grasp_extension',
                'lift_joint2',
                'lift_joint3',
                'lift_joint4',
                'lower_joint1',
                'lower_joint2',
                'lower_joint3',
                'lower_joint4',
                'home_joint1',
                'home_joint2',
                'home_joint3',
                'home_joint4',
                'gripper_open',
                'gripper_close',
                'turn_duration',
                'forward_duration',
                'scan_speed',
                'forward_speed_llm',
            ])
        ]
    )

    return LaunchDescription([
        *launch_args,
        HARDWARE_LAUNCH,
        yolo_publisher_node,
        mission_node,
    ])
