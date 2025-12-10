#!/usr/bin/env python3
"""
Part 3 NLP Launch File (Remote PC - Whisper and Espeak)

This launch file runs on a powerful remote PC and includes:
1. Whisper Publisher - Speech-to-text transcription service
2. Espeak Subscriber - Text-to-speech output service

This node subscribes to /audio_command (receives audio file paths from TurtleBot)
and publishes to /voice_transcription (sends transcribed text back to TurtleBot).

The espeak subscriber listens to /text_to_speech and speaks the received text.

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
    speech_rate:=<int>      Espeak speech rate in words per minute (default: 140)

Examples:
    # Use GPU acceleration
    ros2 launch milestone6 part3.nlp.launch.py device:=cuda
    
    # Use larger model for better accuracy
    ros2 launch milestone6 part3.nlp.launch.py model_type:=medium
    
    # Use tiny model for faster transcription
    ros2 launch milestone6 part3.nlp.launch.py model_type:=tiny
    
    # Adjust speech rate
    ros2 launch milestone6 part3.nlp.launch.py speech_rate:=160
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

    speech_rate_arg = DeclareLaunchArgument(
        'speech_rate',
        default_value='140',
        description='Espeak speech rate in words per minute'
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

    # Espeak Subscriber Node
    espeak_subscriber_node = Node(
        package='milestone6',
        executable='espeak',
        name='espeak',
        output='screen',
        parameters=[{
            'speech_rate': LaunchConfiguration('speech_rate'),
        }]
    )

    return LaunchDescription([
        # Launch arguments
        device_arg,
        model_type_arg,
        temperature_arg,
        speech_rate_arg,
        # Nodes
        whisper_publisher_node,
        espeak_subscriber_node
    ])
