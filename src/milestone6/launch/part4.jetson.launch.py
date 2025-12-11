#!/usr/bin/env python3
"""
Part 4 Remote PC Launch File (LLaMA + Espeak)

This launch file runs on a powerful remote PC and includes:
1. LLaMA Publisher - Takes user text input and translates to atomic robot actions
2. Espeak Subscriber - Text-to-speech output service

The LLaMA publisher subscribes to /user_command (text input from user) and
publishes to /llm_action (atomic actions for the robot).

The espeak subscriber listens to /text_to_speech and speaks the received text.

The Part 4 mission (part4.tb3.launch.py) should be running on the TurtleBot separately.

Workflow:
1. User types natural language commands and publishes them to /user_command:
   ros2 topic pub /user_command std_msgs/String "data: 'Turn around and look for the bottle'"
   
2. LLaMA processes the command and publishes atomic action to /llm_action
   
3. TurtleBot receives action and executes it
   
4. Robot provides feedback via /text_to_speech (spoken by Espeak)

Usage:
    # On Remote PC:
    ros2 launch milestone6 part4.jetson.launch.py
    
    # On TurtleBot (separately):
    ros2 launch milestone6 part4.tb3.launch.py
    
    # Send commands from another terminal:
    ros2 topic pub /user_command std_msgs/String "data: 'Scan for the bottle'"
    ros2 topic pub /user_command std_msgs/String "data: 'Turn left'"
    ros2 topic pub /user_command std_msgs/String "data: 'Move forward'"
    ros2 topic pub /user_command std_msgs/String "data: 'Pick up the bottle'"
    ros2 topic pub /user_command std_msgs/String "data: 'Go to the bear which is 2 meters ahead'"
    ros2 topic pub /user_command std_msgs/String "data: 'Search for the bear'"
    ros2 topic pub /user_command std_msgs/String "data: 'Place the bottle'"
    ros2 topic pub /user_command std_msgs/String "data: 'We are done'"

Optional Parameters:
    model_path:=<str>           Path to LLaMA model (default: uses LlamaBackend default)
    instruct:=<bool>            Use instruct model (default: true)
    n_ctx:=<int>                Context size (default: 512)
    n_threads:=<int>            CPU threads (default: 4)
    n_gpu_layers:=<int>         GPU layers to offload (default: 33)
    temperature:=<float>        Sampling temperature (default: 0.7)
    top_p:=<float>              Top-p sampling (default: 0.95)
    max_tokens:=<int>           Max output tokens (default: 128)
    speech_rate:=<int>          Espeak speech rate in WPM (default: 140)

Examples:
    # Use different temperature for more creative responses
    ros2 launch milestone6 part4.jetson.launch.py temperature:=0.3
    
    # Use more GPU layers for faster inference
    ros2 launch milestone6 part4.jetson.launch.py n_gpu_layers:=50
    
    # Adjust speech rate
    ros2 launch milestone6 part4.jetson.launch.py speech_rate:=160
"""

from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


def generate_launch_description():
    """Generate launch description for Part 4 Remote PC (LLaMA + Espeak)."""

    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument('model_path', default_value='',
                              description='Path to LLaMA model (empty=use default from LlamaBackend)'),
        DeclareLaunchArgument('instruct', default_value='true',
                              description='Use instruct model variant'),
        DeclareLaunchArgument('n_ctx', default_value='512',
                              description='Context window size'),
        DeclareLaunchArgument('n_threads', default_value='4',
                              description='Number of CPU threads'),
        DeclareLaunchArgument('n_gpu_layers', default_value='33',
                              description='Number of GPU layers to offload'),
        DeclareLaunchArgument('temperature', default_value='0.7',
                              description='Sampling temperature (lower=more deterministic)'),
        DeclareLaunchArgument('top_p', default_value='0.95',
                              description='Top-p sampling parameter'),
        DeclareLaunchArgument('max_tokens', default_value='128',
                              description='Maximum tokens in response'),
        DeclareLaunchArgument('speech_rate', default_value='140',
                              description='Espeak speech rate in words per minute'),
    ]

    # LLaMA Publisher Node
    llama_publisher_node = Node(
        package='milestone6',
        executable='llama',
        name='llama',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'instruct': LaunchConfiguration('instruct'),
            'n_ctx': LaunchConfiguration('n_ctx'),
            'n_threads': LaunchConfiguration('n_threads'),
            'n_gpu_layers': LaunchConfiguration('n_gpu_layers'),
            'temperature': LaunchConfiguration('temperature'),
            'top_p': LaunchConfiguration('top_p'),
            'max_tokens': LaunchConfiguration('max_tokens'),
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
        *launch_args,
        # Nodes
        llama_publisher_node,
        espeak_subscriber_node,
    ])
