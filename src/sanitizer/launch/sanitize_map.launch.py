import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# launch power_map and energy_map nodes

def generate_launch_description():

    power_map_node = Node(
        package="sanitizer",
        executable="power_map",
        name="power_mapper",
        output="screen",
        parameters=[],
    )

    energy_map_node = Node(
        package="sanitizer",
        executable="energy_map",
        name="energy_mapper",
        output="screen",
        parameters=[],
    )

    sanitizer_node = Node(
        package="sanitizer",
        executable="sanitizer",
        name="sanitizer",
        output="screen",
        parameters=[],
    )

    ld = LaunchDescription()
    ld.add_action(power_map_node)
    ld.add_action(energy_map_node)
    ld.add_action(sanitizer_node)

    return ld