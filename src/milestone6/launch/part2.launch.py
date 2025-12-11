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
    image_width:=<int>          Camera width (default: 1280)
    image_height:=<int>         Camera height (default: 720)
    display:=<bool>             Show YOLO window (default: true)
    tracking_classes:=<str>     Comma-separated COCO class IDs (default: '39' for bottle)
    bbox_tolerance:=<int>       Bbox width tolerance px (default: 20)
    center_tolerance:=<int>     Centering tolerance px (default: 30)
    target_bbox_width:=<int>    Ideal bbox width (default: 180)
    speed:=<float>              Linear velocity m/s (default: 0.15)
    turn_speed:=<float>         Angular velocity rad/s (default: 1.0)
    detection_timeout:=<float>  Detection timeout seconds (default: 1.0)
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Part 2 mission."""

    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument('include_hardware', default_value='true',
                              description='Include hardware.launch.py (only use when running on TurtleBot itself)'),
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
        DeclareLaunchArgument('tracking_classes', default_value='39',
                              description='Comma-separated COCO class IDs to track (e.g., "39,41" for bottle and cup)'),
        DeclareLaunchArgument('bbox_tolerance', default_value='20',
                              description='Bounding box width tolerance in pixels'),
        DeclareLaunchArgument('center_tolerance', default_value='30',
                              description='Centering tolerance in pixels'),
        DeclareLaunchArgument('target_bbox_width', default_value='365',
                              description='Ideal bounding box width for grabbing in pixels'),
        DeclareLaunchArgument('speed', default_value='0.05',
                              description='Linear velocity in m/s'),
        DeclareLaunchArgument('turn_speed', default_value='0.25',
                              description='Angular velocity in rad/s'),
        DeclareLaunchArgument('transport_distance', default_value='0.10',
                              description='Distance to transport object in meters'),
        DeclareLaunchArgument('detection_timeout', default_value='1.0',
                              description='Detection timeout in seconds'),
        # Arm joint position arguments
        DeclareLaunchArgument('grab_joint2', default_value='0.95',
                              description='Grab position: forward reach (radians)'),
        DeclareLaunchArgument('grab_joint3', default_value='-0.65',
                              description='Grab position: extend elbow (radians)'),
        DeclareLaunchArgument('grab_joint4', default_value='0.0',
                              description='Grab position: level gripper (radians)'),
        DeclareLaunchArgument('grab_vertical_adjust', default_value='0.2',
                              description='Extra reach for low objects (radians)'),
        DeclareLaunchArgument('grasp_extension', default_value='0.2',
                              description='Extra extension to grasp (radians)'),
        DeclareLaunchArgument('lift_joint2', default_value='-0.5',
                              description='Lift position: lift up (radians)'),
        DeclareLaunchArgument('lift_joint3', default_value='0.4',
                              description='Lift position: retract (radians)'),
        DeclareLaunchArgument('lift_joint4', default_value='0.6',
                              description='Lift position: adjust wrist (radians)'),
        DeclareLaunchArgument('lower_joint1', default_value='0.0',
                              description='Lower position: center (radians)'),
        DeclareLaunchArgument('lower_joint2', default_value='0.6',
                              description='Lower position: down (radians)'),
        DeclareLaunchArgument('lower_joint3', default_value='-0.4',
                              description='Lower position: extend (radians)'),
        DeclareLaunchArgument('lower_joint4', default_value='0.6',
                              description='Lower position: level (radians)'),
        DeclareLaunchArgument('home_joint1', default_value='0.0',
                              description='Home position: joint1 (radians)'),
        DeclareLaunchArgument('home_joint2', default_value='-1.05',
                              description='Home position: joint2 (radians)'),
        DeclareLaunchArgument('home_joint3', default_value='0.35',
                              description='Home position: joint3 (radians)'),
        DeclareLaunchArgument('home_joint4', default_value='0.70',
                              description='Home position: joint4 (radians)'),
        DeclareLaunchArgument('gripper_open', default_value='0.025',
                              description='Gripper open position'),
        DeclareLaunchArgument('gripper_close', default_value='-0.015',
                              description='Gripper close position'),
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

    # Part 2 Mission Node
    mission_node = Node(
        package='milestone6',
        executable='part2',
        name='part2',
        output='screen',
        parameters=[{
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
        # Hardware (conditional)
        hardware_launch,
        # Nodes
        yolo_publisher_node,
        mission_node
    ])
