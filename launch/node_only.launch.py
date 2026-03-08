#!/usr/bin/env python3
"""
node_only.launch.py
===================
Launch ONLY the color_search_node for quick testing.
Assumes Gazebo + Nav2 are already running in separate terminals.

Usage:
  ros2 launch red_ball_nav node_only.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    color_search_node = Node(
        package="red_ball_nav",
        executable="color_search_node",
        name="color_search_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time":     use_sim_time},
            {"rotate_speed":     0.4},         # rad/s — tweak as needed
            {"goal_x":           1.5},          # meters
            {"goal_y":           0.0},
            {"goal_yaw":         0.0},          # radians
            {"min_contour_area": 500},          # pixels²
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        color_search_node,
    ])
