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
    image_width:=<int>          Camera width (default: 500)
    image_height:=<int>         Camera height (default: 320)
    display:=<bool>             Show YOLO window (default: true)
    tracking_classes:=<str>     Comma-separated COCO class IDs (default: '39' for bottle)
    center_tolerance:=<int>     Centering tolerance px (default: 30)
    target_bbox_width:=<int>    Ideal bbox width (default: 180)
    forward_speed:=<float>      Linear velocity m/s (default: 0.15)
    turn_speed:=<float>         Angular velocity rad/s (default: 1.0)
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
        description='Display YOLO detection window'
    )

    tracking_classes_arg = DeclareLaunchArgument(
        'tracking_classes',
        default_value='39',
        description='Comma-separated COCO class IDs to track (e.g., "39,41" for bottle and cup)'
    )

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
        description='Linear velocity in m/s'
    )

    turn_speed_arg = DeclareLaunchArgument(
        'turn_speed',
        default_value='0.25',
        description='Angular velocity in rad/s'
    )

    transport_distance_arg = DeclareLaunchArgument(
        'transport_distance',
        default_value='0.10',
        description='Distance to transport object in meters (3.2ft = 0.975m)'
    )

    detection_timeout_arg = DeclareLaunchArgument(
        'detection_timeout',
        default_value='1',
        description='Detection timeout in seconds'
    )

    # Arm joint position arguments
    grab_joint2_arg = DeclareLaunchArgument(
        'grab_joint2', default_value='0.95',
        description='Grab position: forward reach (radians)'
    )
    grab_joint3_arg = DeclareLaunchArgument(
        'grab_joint3', default_value='-0.65',
        description='Grab position: extend elbow (radians)'
    )
    grab_joint4_arg = DeclareLaunchArgument(
        'grab_joint4', default_value='0.0',
        description='Grab position: level gripper (radians)'
    )
    grab_vertical_adjust_arg = DeclareLaunchArgument(
        'grab_vertical_adjust', default_value='0.2',
        description='Extra reach for low objects (radians)'
    )
    grasp_extension_arg = DeclareLaunchArgument(
        'grasp_extension', default_value='0.2',
        description='Extra extension to grasp (radians)'
    )

    lift_joint2_arg = DeclareLaunchArgument(
        'lift_joint2', default_value='-0.5',
        description='Lift position: lift up (radians)'
    )
    lift_joint3_arg = DeclareLaunchArgument(
        'lift_joint3', default_value='0.4',
        description='Lift position: retract (radians)'
    )
    lift_joint4_arg = DeclareLaunchArgument(
        'lift_joint4', default_value='0.6',
        description='Lift position: adjust wrist (radians)'
    )

    lower_joint1_arg = DeclareLaunchArgument(
        'lower_joint1', default_value='0.0',
        description='Lower position: center (radians)'
    )
    lower_joint2_arg = DeclareLaunchArgument(
        'lower_joint2', default_value='0.6',
        description='Lower position: down (radians)'
    )
    lower_joint3_arg = DeclareLaunchArgument(
        'lower_joint3', default_value='-0.4',
        description='Lower position: extend (radians)'
    )
    lower_joint4_arg = DeclareLaunchArgument(
        'lower_joint4', default_value='0.6',
        description='Lower position: level (radians)'
    )

    home_joint1_arg = DeclareLaunchArgument(
        'home_joint1', default_value='0.0',
        description='Home position: joint1 (radians)'
    )
    home_joint2_arg = DeclareLaunchArgument(
        'home_joint2', default_value='-1.05',
        description='Home position: joint2 (radians)'
    )
    home_joint3_arg = DeclareLaunchArgument(
        'home_joint3', default_value='0.35',
        description='Home position: joint3 (radians)'
    )
    home_joint4_arg = DeclareLaunchArgument(
        'home_joint4', default_value='0.70',
        description='Home position: joint4 (radians)'
    )

    gripper_open_arg = DeclareLaunchArgument(
        'gripper_open', default_value='0.025',
        description='Gripper open position'
    )
    gripper_close_arg = DeclareLaunchArgument(
        'gripper_close', default_value='-0.015',
        description='Gripper close position'
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

    # Part 2 Mission Node
    mission_node = Node(
        package='milestone6',
        executable='part2',
        name='part2',
        output='screen',
        parameters=[{
            'tracking_classes': LaunchConfiguration('tracking_classes'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'bbox_tolerance': 20,
            'center_tolerance': LaunchConfiguration('center_tolerance'),
            'target_bbox_width': LaunchConfiguration('target_bbox_width'),
            'forward_speed': LaunchConfiguration('forward_speed'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'detection_timeout': LaunchConfiguration('detection_timeout'),
            'transport_distance': LaunchConfiguration('transport_distance'),
            # Arm joint positions
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
        include_hardware_arg,
        yolo_model_arg,
        image_width_arg,
        image_height_arg,
        display_arg,
        tracking_classes_arg,
        center_tolerance_arg,
        target_bbox_width_arg,
        forward_speed_arg,
        turn_speed_arg,
        transport_distance_arg,
        detection_timeout_arg,
        # Arm joint arguments
        grab_joint2_arg,
        grab_joint3_arg,
        grab_joint4_arg,
        grab_vertical_adjust_arg,
        grasp_extension_arg,
        lift_joint2_arg,
        lift_joint3_arg,
        lift_joint4_arg,
        lower_joint1_arg,
        lower_joint2_arg,
        lower_joint3_arg,
        lower_joint4_arg,
        home_joint1_arg,
        home_joint2_arg,
        home_joint3_arg,
        home_joint4_arg,
        gripper_open_arg,
        gripper_close_arg,
        # Hardware (conditional)
        hardware_launch,
        # Nodes
        yolo_publisher_node,
        mission_node
    ])
