#!/usr/bin/env python3
"""Utility functions and classes for launch files."""

from dataclasses import dataclass

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

HARDWARE_LAUNCH = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare('turtlebot3_manipulation_bringup'),
        '/launch/hardware.launch.py'
    ]),
    launch_arguments={
        'log_level': 'error'
    }.items(),
    condition=IfCondition(LaunchConfiguration('include_hardware'))
)


@dataclass
class LaunchArg:
    """Dataclass for launch argument configuration."""
    key: str
    value: str
    description: str = ''

    def to_declare_launch_argument(self) -> DeclareLaunchArgument:
        """Convert to DeclareLaunchArgument."""
        return DeclareLaunchArgument(
            self.key,
            default_value=self.value,
            description=self.description
        )

    @staticmethod
    def generate_launch_arguments(*arg_dicts) -> list[DeclareLaunchArgument]:
        """Convert one or more LAUNCH_ARGS dictionaries to DeclareLaunchArgument list.

        Args:
            *arg_dicts: Variable number of dictionaries with format:
                       {key: (default_value, description), ...}

        Returns:
            List of DeclareLaunchArgument objects.
        """
        result = []
        for arg_dict in arg_dicts:
            for key, descriptor in arg_dict.items():
                # Support both default_value only and (default_value, description) tuple
                if isinstance(descriptor, tuple):
                    value, description = descriptor
                else:
                    value = descriptor
                    description = ''
                result.append(DeclareLaunchArgument(
                    key,
                    default_value=value,
                    description=description
                ))
        return result

    @staticmethod
    def generate_launch_configs(keys: list[str]) -> dict[str, LaunchConfiguration]:
        """Convert a list of keys to a dictionary mapping keys to LaunchConfiguration objects.

        Args:
            keys: List of launch argument keys.

        Returns:
            Dictionary mapping each key to LaunchConfiguration(key).
        """
        return {key: LaunchConfiguration(key) for key in keys}
