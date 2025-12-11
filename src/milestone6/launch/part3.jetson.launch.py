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
    model:=<str>            Whisper model type (tiny, base, small, medium, large, default: tiny.en)
    temperature:=<float>    Sampling temperature (default: 0.0)
    timer_period:=<float>   Timer period for listening loop in seconds (default: 1.0)
    audio_sample_rate:=<int> Audio sample rate in Hz (default: 16000)
    audio_duration:=<float> Audio recording duration in seconds (default: 5.0)
    audio_threshold:=<int>  Amplitude threshold for voice activity detection (default: 800)
    audio_file:=<str>       Path to save audio recordings (default: /tmp/whisper_audio.wav)
    speech_rate:=<int>      Espeak speech rate in words per minute (default: 140)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from milestone6.util.launch import LaunchArg

LAUNCH_ARGS = {
    'device': ('cpu', 'Device for Whisper inference (cpu or cuda)'),
    'model': ('tiny.en', 'Whisper model type (tiny, base, small, medium, large)'),
    'temperature': ('0.0', 'Whisper sampling temperature'),
    'timer_period': ('1.0', 'Timer period for listening loop in seconds'),
    'audio_sample_rate': ('16000', 'Audio sample rate in Hz'),
    'audio_duration': ('5.0', 'Audio recording duration in seconds'),
    'audio_threshold': ('800', 'Amplitude threshold for voice activity detection'),
    'audio_file': ('/tmp/whisper_audio.wav', 'Path to save audio recordings'),
    'speech_rate': ('140', 'Espeak speech rate in words per minute'),
}


def generate_launch_description():
    """Generate launch description for Part 3 NLP (Whisper only)."""
    # Generate launch arguments
    launch_args = LaunchArg.generate_launch_arguments(LAUNCH_ARGS)

    # Whisper Publisher Node
    whisper_publisher_node = Node(
        package='milestone6',
        executable='whisper',
        name='whisper',
        output='screen',
        parameters=[
            LaunchArg.generate_launch_configs([
                'device',
                'model',
                'temperature',
                'timer_period',
                'audio_sample_rate',
                'audio_duration',
                'audio_threshold',
                'audio_file',
            ])
        ]
    )

    # Espeak Subscriber Node
    espeak_subscriber_node = Node(
        package='milestone6',
        executable='espeak',
        name='espeak',
        output='screen',
        parameters=[
            LaunchArg.generate_launch_configs([
                'speech_rate',
            ])
        ]
    )

    return LaunchDescription([
        *launch_args,
        whisper_publisher_node,
        espeak_subscriber_node
    ])
