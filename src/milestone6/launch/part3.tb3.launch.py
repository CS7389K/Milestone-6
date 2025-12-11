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
    turn_duration:=<float>         Duration for turn commands in seconds (default: 1.0)
    forward_duration:=<float>      Duration for forward commands in seconds (default: 0.5)
    scan_speed:=<float>            Rotation speed for scan command in rad/s (default: 0.5)
    audio_file:=<str>              Path to save audio recordings (default: /tmp/voice_input.wav)
    audio_duration:=<int>          Audio recording duration in seconds (default: 5)
    audio_sample_rate:=<int>       Audio sample rate in Hz (default: 16000)
    audio_threshold:=<int>         Amplitude threshold for voice activity detection (default: 800)
    prompt_delay:=<float>          Time between voice prompts in seconds (default: 30.0)
    speed:=<float>                 Linear movement speed in m/s (default: 0.05)
    bbox_tolerance:=<int>          Bounding box width tolerance in pixels (default: 20)
    center_tolerance:=<int>        Centering tolerance in pixels (default: 30)
    target_bbox_width:=<int>       Target bounding box width for approach in pixels (default: 365)
    turn_speed:=<float>            Angular velocity for turning rad/s (default: 0.25)
    detection_timeout:=<float>     Stop if no detection for N seconds (default: 1.0)
    transport_distance:=<float>    Distance to transport object in meters (default: 0.10)
    grab_joint2:=<float>           Grab position: forward reach (radians) (default: 0.95)
    grab_joint3:=<float>           Grab position: extend elbow (radians) (default: -0.65)
    grab_joint4:=<float>           Grab position: level gripper (radians) (default: 0.0)
    grab_vertical_adjust:=<float>  Extra reach for low objects (radians) (default: 0.2)
    grasp_extension:=<float>       Extra extension to grasp (radians) (default: 0.2)
    lift_joint2:=<float>           Lift position: lift up (radians) (default: -0.5)
    lift_joint3:=<float>           Lift position: retract (radians) (default: 0.4)
    lift_joint4:=<float>           Lift position: adjust wrist (radians) (default: 0.6)
    lower_joint1:=<float>          Lower position: center (radians) (default: 0.0)
    lower_joint2:=<float>          Lower position: down (radians) (default: 0.6)
    lower_joint3:=<float>          Lower position: extend (radians) (default: -0.4)
    lower_joint4:=<float>          Lower position: level (radians) (default: 0.6)
    home_joint1:=<float>           Home position: joint1 (radians) (default: 0.0)
    home_joint2:=<float>           Home position: joint2 (radians) (default: -1.05)
    home_joint3:=<float>           Home position: joint3 (radians) (default: 0.35)
    home_joint4:=<float>           Home position: joint4 (radians) (default: 0.70)
    gripper_open:=<float>          Gripper open position (default: 0.025)
    gripper_close:=<float>         Gripper close position (default: -0.015)
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
    'audio_file': ('/tmp/voice_input.wav', 'Path to save audio recordings'),
    'prompt_delay': ('30.0', 'Time between voice prompts in seconds'),
}


def generate_launch_description():
    """Generate launch description for Part 3 base (TurtleBot side)."""
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

    # Part 3 Mission Node
    mission_node = Node(
        package='milestone6',
        executable='part3',
        name='part3',
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
                'audio_file',
                'prompt_delay',
            ])
        ]
    )

    return LaunchDescription([
        *launch_args,
        HARDWARE_LAUNCH,
        yolo_publisher_node,
        mission_node
    ])
