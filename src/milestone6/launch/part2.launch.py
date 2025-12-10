# #!/usr/bin/env python3
# """
# Launch file for Milestone 6 Part 2: TeleopPublisher + YOLO Publisher + TeleopBase + TeleopArm

# Launches:
# - TeleopPublisher Node: Centralized teleop service for base and arm control
# - YOLO Publisher Node: Captures camera frames and publishes object detections
# - TeleopBase Node: Subscribes to detections and controls robot base movement
# - TeleopArm Node: Automatically grabs detected objects

# Usage:
#     ros2 launch milestone6 part2.launch.py

# Optional Parameters:
#     yolo_model:=<path>     Path to YOLO model file (default: yolo11n.pt)
#     image_width:=<int>     Camera image width (default: 500)
#     image_height:=<int>    Camera image height (default: 320)
#     display:=<bool>        Display YOLO detections window (default: true)
#     track_class:=<int>     COCO class ID to track (default: 39 for bottle)
#     move_wheels:=<bool>    Enable robot movement (default: true)
#     tolerance:=<int>       Centering tolerance in pixels (default: 50)
#     min_bbox_width:=<int>  Min bbox width to stop (default: 150)
#     forward_speed:=<float> Linear velocity (default: 0.15)
#     turn_speed:=<float>    Angular velocity (default: 1.5)
# """

# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#     """Generate launch description with YOLO publisher, TeleopPublisher, TeleopBase, and TeleopArm nodes."""    
#     # YOLO Publisher Node
#     yolo_publisher_node = Node(
#         package='milestone6',
#         executable='yolo_publisher',
#         name='yolo_publisher',
#         output='screen',
#         parameters=[{
#             'yolo_model': 'yolo11n.pt',
#             'image_width': 500,
#             'image_height': 320,
#             'display': True,
#         }]
#     )
#     # Teleop Publisher Node
#     teleop_publisher_node = Node(
#         package='milestone6',
#         executable='teleop_publisher',
#         name='teleop_publisher',
#         output='screen',
#     )
#     # TeleopBase Node
#     base_node = Node(
#         package='milestone6',
#         executable='base',
#         name='teleop_base',
#         output='screen',
#         parameters=[{
#             'move_wheels': True,
#             'image_width': 500,
#             'image_height': 320,
#             'tolerance': 50,
#             'min_bbox_width': 150,
#             'forward_speed': 0.15,
#             'turn_speed': 1.5,
#             'track_class': 39,  # bottle
#         }]
#     )
#     # TeleopArm Node
#     arm_node = Node(
#         package='milestone6',
#         executable='arm',
#         name='teleop_arm',
#         output='screen',
#         parameters=[{
#             'target_class': 39,  # bottle
#             'detection_timeout_sec': 1.0,
#             'grab_delay_sec': 0.5,
#         }]
#     )
#     return LaunchDescription([
#         teleop_publisher_node,
#         yolo_publisher_node,
#         base_node,
#         arm_node,
#     ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    arm_mode_arg = DeclareLaunchArgument(
        'arm_mode',
        default_value='arm1',  # arm / arm1 / arm2
        description='Which arm node to run'
    )

    arm_exe = LaunchConfiguration('arm_mode')

    teleop_publisher_node = Node(
        package='milestone6',
        executable='teleop_publisher',
        name='teleop_publisher',
        output='screen',
    )

    yolo_publisher_node = Node(
        package='milestone6',
        executable='yolo_publisher',
        name='yolo_publisher',
        output='screen',
        parameters=[{
            'yolo_model': 'yolo11n.pt',
            'image_width': 500,
            'image_height': 320,
            'display': True,
        }],
        additional_env={'LD_PRELOAD': '/usr/lib/aarch64-linux-gnu/libgomp.so.1'}
    )

    base_node = Node(
        package='milestone6',
        executable='base',
        name='teleop_base',
        output='screen',
        parameters=[{
            'move_wheels': True,
            'image_width': 500,
            'image_height': 320,
            'tolerance': 50,
            'min_bbox_width': 150,
            'forward_speed': 0.15,
            'turn_speed': 1.5,
            'track_class': 39,
        }]
    )

    arm_node = Node(
        package='milestone6',
        executable=arm_exe,
        name='teleop_arm_selected',
        output='screen',
        parameters=[{
            'target_class': 39,
            'detection_timeout_sec': 1.0,
            'grab_delay_sec': 0.5,
        }]
    )

    return LaunchDescription([
        arm_mode_arg,
        teleop_publisher_node,
        yolo_publisher_node,
        base_node,
        arm_node,
    ])

