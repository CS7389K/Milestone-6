#!/usr/bin/env python3
"""
Part 3 NLP Launch File (Remote PC - Whisper Only)

This launch file runs on a powerful remote PC and includes:
1. Whisper Publisher - Speech-to-text transcription service

This node subscribes to /audio_command (receives audio file paths from TurtleBot)
and publishes to /voice_transcription (sends transcribed text back to TurtleBot).

The base mission (part3.base.launch.py) should be running on the TurtleBot separately.

Usage:
    # On Remote PC:
    ros2 launch milestone6 part3.nlp.launch.py
    
    # On TurtleBot (separately):
    ros2 launch milestone6 part3.base.launch.py

Optional Parameters:
    device:=<str>           Device for Whisper (cpu or cuda, default: cpu)
    model_type:=<str>       Whisper model (tiny, base, small, medium, large, default: small)
    temperature:=<float>    Sampling temperature (default: 0.0)

Examples:
    # Use GPU acceleration
    ros2 launch milestone6 part3.nlp.launch.py device:=cuda
    
    # Use larger model for better accuracy
    ros2 launch milestone6 part3.nlp.launch.py model_type:=medium
    
    # Use tiny model for faster transcription
    ros2 launch milestone6 part3.nlp.launch.py model_type:=tiny
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Part 3 NLP (Whisper only)."""

    # Declare launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='Device for Whisper inference (cpu or cuda)'
    )

    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='small',
        description='Whisper model type (tiny, base, small, medium, large)'
    )

    temperature_arg = DeclareLaunchArgument(
        'temperature',
        default_value='0.0',
        description='Whisper sampling temperature'
    )

    # Whisper Publisher Node
    whisper_publisher_node = Node(
        package='milestone6',
        executable='whisper_publisher',
        name='whisper_publisher',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('device'),
            'model_type': LaunchConfiguration('model_type'),
            'temperature': LaunchConfiguration('temperature'),
        }]
    )

    return LaunchDescription([
        # Launch arguments
        device_arg,
        model_type_arg,
        temperature_arg,
        # Whisper publisher node
        whisper_publisher_node
    ])
