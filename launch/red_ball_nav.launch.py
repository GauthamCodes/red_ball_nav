#!/usr/bin/env python3
"""
red_ball_nav.launch.py
======================
Launches:
  1. TurtleBot3 Gazebo world (with a red ball spawned)
  2. Nav2 navigation stack (with AMCL localisation)
  3. color_search_node (our custom node)

Usage:
  ros2 launch red_ball_nav red_ball_nav.launch.py

Before running, set:
  export TURTLEBOT3_MODEL=burger
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # ── Paths ──────────────────────────────────────────────────────────────
    tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # ── Launch Arguments ───────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (Gazebo) clock"
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            tb3_gazebo_dir, "worlds", "turtlebot3_world.world"
        ),
        description="Full path to Gazebo world file",
    )
    map_arg = DeclareLaunchArgument(
        "map",
        # Replace with your own map yaml if needed
        default_value=os.path.join(
            nav2_bringup_dir, "maps", "turtlebot3_world.yaml"
        ),
        description="Full path to map yaml file",
    )
    nav2_params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            nav2_bringup_dir, "params", "nav2_params.yaml"
        ),
        description="Full path to Nav2 params yaml",
    )

    use_sim_time  = LaunchConfiguration("use_sim_time")
    world         = LaunchConfiguration("world")
    map_yaml      = LaunchConfiguration("map")
    params_file   = LaunchConfiguration("params_file")

    # ── 1. Gazebo simulation ────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, "launch", "turtlebot3_world.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": world,
        }.items(),
    )

    # ── 2. Spawn red ball in Gazebo ─────────────────────────────────────────
    spawn_red_ball = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_red_ball",
        arguments=[
            "-entity", "red_ball",
            "-file",   "/dev/stdin",    # we pass SDF inline via stdin workaround
            # Alternatively use: -x 1.5 -y 0.0 -z 0.1
            # Below we use a simple sphere SDF string via Python
        ],
        output="screen",
        # NOTE: Replace with path to your red_ball.sdf (see below for SDF content)
        # Better approach: use the sdf file from the package
    )
    # ── Nicer approach: spawn via topic using a pre-written SDF file ────────
    # We create the SDF file and spawn it (see red_ball.sdf created separately)
    spawn_red_ball_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_red_ball",
        arguments=[
            "-entity", "red_ball",
            "-file",
            PathJoinSubstitution([
                get_package_share_directory("red_ball_nav"),
                "..",  # adjusted below
            ]),
            "-x", "1.5",
            "-y", "0.0",
            "-z", "0.1",
        ],
        output="screen",
    )

    # ── 3. Nav2 Navigation stack ────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map":          map_yaml,
            "params_file":  params_file,
        }.items(),
    )

    # ── 4. Our custom color_search_node (delayed 10 s for Nav2 to boot) ────
    color_search = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="red_ball_nav",
                executable="color_search_node",
                name="color_search_node",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"rotate_speed":     0.4},
                    {"goal_x":           1.5},
                    {"goal_y":           0.0},
                    {"goal_yaw":         0.0},
                    {"min_contour_area": 500},
                ],
            )
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        map_arg,
        nav2_params_arg,
        gazebo_launch,
        nav2_launch,
        color_search,
    ])
