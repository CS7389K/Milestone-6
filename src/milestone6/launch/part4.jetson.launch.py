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
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from milestone6.util.launch import LaunchArg

LAUNCH_ARGS = {
    'model_path': ('', 'Path to LLaMA model (empty=use default from LlamaBackend)'),
    'instruct': ('true', 'Use instruct model variant'),
    'n_ctx': ('512', 'Context window size'),
    'n_threads': ('4', 'Number of CPU threads'),
    'n_gpu_layers': ('33', 'Number of GPU layers to offload'),
    'temperature': ('0.7', 'Sampling temperature (lower=more deterministic)'),
    'top_p': ('0.95', 'Top-p sampling parameter'),
    'max_tokens': ('128', 'Maximum tokens in response'),
    'speech_rate': ('140', 'Espeak speech rate in words per minute'),
}


def generate_launch_description():
    """Generate launch description for Part 4 Remote PC (LLaMA + Espeak)."""
    # Generate launch arguments
    launch_args = LaunchArg.generate_launch_arguments(LAUNCH_ARGS)

    # cli_node = Node(
    #     package='milestone6',
    #     executable='cli',
    #     name='cli',
    #     output='screen'
    # )

    # LLaMA Publisher Node
    llama_publisher_node = Node(
        package='milestone6',
        executable='llama',
        name='llama',
        output='screen',
        parameters=[
            LaunchArg.generate_launch_configs([
                'model_path',
                'instruct',
                'n_ctx',
                'n_threads',
                'n_gpu_layers',
                'temperature',
                'top_p',
                'max_tokens',
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
        # cli_node,
        llama_publisher_node,
        espeak_subscriber_node,
    ])
