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
    include_hardware_arg = DeclareLaunchArgument(
        'include_hardware',
        default_value='true',
        description='Include hardware.launch.py (only use when running on TurtleBot itself)'
    )

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
        description='Display YOLO detection window (usually false on headless TurtleBot)'
    )

    tracking_classes_arg = DeclareLaunchArgument(
        'tracking_classes',
        default_value='39',
        description='Comma-separated COCO class IDs to track (e.g., "39" for bottle)'
    )

    # Voice search parameters
    turn_duration_arg = DeclareLaunchArgument(
        'turn_duration',
        default_value='1.0',
        description='Duration for turn commands in seconds'
    )

    forward_duration_arg = DeclareLaunchArgument(
        'forward_duration',
        default_value='0.5',
        description='Duration for forward commands in seconds'
    )

    scan_speed_arg = DeclareLaunchArgument(
        'scan_speed',
        default_value='0.5',
        description='Rotation speed for scan command in rad/s'
    )

    turn_speed_voice_arg = DeclareLaunchArgument(
        'turn_speed_voice',
        default_value='1.0',
        description='Angular velocity for voice turn commands in rad/s'
    )

    forward_speed_voice_arg = DeclareLaunchArgument(
        'forward_speed_voice',
        default_value='0.2',
        description='Linear velocity for voice forward commands in m/s'
    )

    audio_file_arg = DeclareLaunchArgument(
        'audio_file',
        default_value='/tmp/voice_input.wav',
        description='Path to save audio recordings'
    )

    audio_duration_arg = DeclareLaunchArgument(
        'audio_duration',
        default_value='5',
        description='Audio recording duration in seconds'
    )

    audio_sample_rate_arg = DeclareLaunchArgument(
        'audio_sample_rate',
        default_value='16000',
        description='Audio sample rate in Hz'
    )

    audio_threshold_arg = DeclareLaunchArgument(
        'audio_threshold',
        default_value='500',
        description='Amplitude threshold for voice activity detection'
    )

    # Visual servoing parameters
    center_tolerance_arg = DeclareLaunchArgument(
        'center_tolerance',
        default_value='30',
        description='Centering tolerance in pixels'
    )

    target_bbox_width_arg = DeclareLaunchArgument(
        'target_bbox_width',
        default_value='365',
        description='Ideal bounding box width for grabbing in pixels'
    )

    forward_speed_arg = DeclareLaunchArgument(
        'forward_speed',
        default_value='0.05',
        description='Linear velocity in m/s for visual servoing'
    )

    turn_speed_arg = DeclareLaunchArgument(
        'turn_speed',
        default_value='0.25',
        description='Angular velocity in rad/s for visual servoing'
    )

    transport_distance_arg = DeclareLaunchArgument(
        'transport_distance',
        default_value='0.10',
        description='Distance to transport object in meters'
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

    # Part 3 Mission Node (with Whisper subscriber enabled)
    mission_node = Node(
        package='milestone6',
        executable='part3',
        name='part3',
        output='screen',
        parameters=[{
            # Whisper subscriber enabled
            'use_whisper': True,
            # Voice search parameters
            'turn_duration': LaunchConfiguration('turn_duration'),
            'forward_duration': LaunchConfiguration('forward_duration'),
            'scan_speed': LaunchConfiguration('scan_speed'),
            'turn_speed_voice': LaunchConfiguration('turn_speed_voice'),
            'forward_speed_voice': LaunchConfiguration('forward_speed_voice'),
            'audio_file': LaunchConfiguration('audio_file'),
            'prompt_delay': 30.0,
            'audio_duration': LaunchConfiguration('audio_duration'),
            'audio_sample_rate': LaunchConfiguration('audio_sample_rate'),
            'audio_threshold': LaunchConfiguration('audio_threshold'),
            # Visual servoing parameters
            'tracking_classes': LaunchConfiguration('tracking_classes'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'bbox_tolerance': 20,
            'center_tolerance': LaunchConfiguration('center_tolerance'),
            'target_bbox_width': LaunchConfiguration('target_bbox_width'),
            'forward_speed': LaunchConfiguration('forward_speed'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'detection_timeout': 1,
            'transport_distance': LaunchConfiguration('transport_distance'),
            # Arm joint positions (using Part 2 defaults)
            'grab_joint2': 0.95,
            'grab_joint3': -0.65,
            'grab_joint4': 0.0,
            'grab_vertical_adjust': 0.2,
            'grasp_extension': 0.2,
            'lift_joint2': -0.5,
            'lift_joint3': 0.4,
            'lift_joint4': 0.6,
            'lower_joint1': 0.0,
            'lower_joint2': 0.6,
            'lower_joint3': -0.4,
            'lower_joint4': 0.6,
            'home_joint1': 0.0,
            'home_joint2': -1.05,
            'home_joint3': 0.35,
            'home_joint4': 0.70,
            'gripper_open': 0.025,
            'gripper_close': -0.015,
        }]
    )

    return LaunchDescription([
        # Launch arguments
        include_hardware_arg,
        yolo_model_arg,
        image_width_arg,
        image_height_arg,
        display_arg,
        tracking_classes_arg,
        turn_duration_arg,
        forward_duration_arg,
        scan_speed_arg,
        turn_speed_voice_arg,
        forward_speed_voice_arg,
        audio_file_arg,
        audio_duration_arg,
        audio_sample_rate_arg,
        audio_threshold_arg,
        center_tolerance_arg,
        target_bbox_width_arg,
        forward_speed_arg,
        turn_speed_arg,
        transport_distance_arg,
        # Hardware (conditional)
        hardware_launch,
        # Nodes
        yolo_publisher_node,
        mission_node
    ])
