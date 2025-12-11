#!/usr/bin/env python3
"""
Part 3 Base Launch File (TurtleBot Side)

This launch file runs on the TurtleBot and includes:
1. Hardware bringup (arm and base controllers)
2. YOLO Publisher (object detection)
3. Part 3 Mission Node (voice-guided search, refactored to use Whisper subscriber)

This node subscribes to /voice_transcription from a remote Whisper publisher.
The Whisper publisher should be run separately on a powerful PC using part3.nlp.launch.py

Usage:
    # On TurtleBot:
    ros2 launch milestone6 part3.base.launch.py
    
    # On Remote PC (separately):
    ros2 launch milestone6 part3.nlp.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Part 3 base (TurtleBot side)."""

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
                              description='Display YOLO detection window (usually false on headless TurtleBot)'),
        DeclareLaunchArgument('camera_backend', default_value='gstreamer',
                              description='Camera backend (gstreamer, opencv)'),
        DeclareLaunchArgument('camera_device', default_value='1',
                              description='Camera device ID'),
        DeclareLaunchArgument('gstreamer_pipeline', default_value='',
                              description='Custom GStreamer pipeline'),
        DeclareLaunchArgument('tracking_classes', default_value='39',
                              description='Comma-separated COCO class IDs to track (e.g., "39" for bottle)'),
        # Voice search parameters
        DeclareLaunchArgument('turn_duration', default_value='1.0',
                              description='Duration for turn commands in seconds'),
        DeclareLaunchArgument('forward_duration', default_value='0.5',
                              description='Duration for forward commands in seconds'),
        DeclareLaunchArgument('scan_speed', default_value='0.5',
                              description='Rotation speed for scan command in rad/s'),
        DeclareLaunchArgument('audio_file', default_value='/tmp/voice_input.wav',
                              description='Path to save audio recordings'),
        DeclareLaunchArgument('audio_duration', default_value='5',
                              description='Audio recording duration in seconds'),
        DeclareLaunchArgument('audio_sample_rate', default_value='16000',
                              description='Audio sample rate in Hz'),
        DeclareLaunchArgument('audio_threshold', default_value='800',
                              description='Amplitude threshold for voice activity detection'),
        DeclareLaunchArgument('prompt_delay', default_value='30.0',
                              description='Time between voice prompts in seconds'),
        # Visual servoing parameters
        DeclareLaunchArgument('bbox_tolerance', default_value='20',
                              description='Bounding box width tolerance in pixels'),
        DeclareLaunchArgument('center_tolerance', default_value='30',
                              description='Centering tolerance in pixels'),
        DeclareLaunchArgument('target_bbox_width', default_value='365',
                              description='Ideal bounding box width for grabbing in pixels'),
        DeclareLaunchArgument('speed', default_value='0.05',
                              description='Linear velocity in m/s for visual servoing'),
        DeclareLaunchArgument('turn_speed', default_value='0.25',
                              description='Angular velocity in rad/s for visual servoing'),
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

    # Part 3 Mission Node (with Whisper subscriber enabled)
    mission_node = Node(
        package='milestone6',
        executable='part3',
        name='part3',
        output='screen',
        parameters=[{
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
        # Hardware (conditional)
        hardware_launch,
        # Nodes
        yolo_publisher_node,
        mission_node
    ])
