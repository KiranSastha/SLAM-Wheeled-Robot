#!/usr/bin/env python3
"""
slam_nav2_rrtstar.launch.py
Launches Gazebo + SLAM Toolbox + Nav2 (with RRT*) in the correct
order for ROS2 Jazzy, using staggered TimerActions.

Usage:
  ros2 launch rrt_star_planner slam_nav2_rrtstar.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_rrt = get_package_share_directory("rrt_star_planner")
    pkg_gazebo = get_package_share_directory("turtlebot3_gazebo")
    pkg_slam = get_package_share_directory("slam_toolbox")
    pkg_nav2 = get_package_share_directory("turtlebot3_navigation2")

    nav2_params_path = os.path.join(pkg_rrt, "config", "nav2_params_rrtstar.yaml")

    # ── 1. Gazebo simulation (starts immediately) ────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "turtlebot3_world.launch.py")
        )
    )

    # ── 2. SLAM Toolbox (starts after 6s, once Gazebo clock is live) ──
    slam = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam, "launch", "online_async_launch.py")
                ),
                launch_arguments=[
                    ("use_sim_time", "True"),
                ],
            )
        ],
    )

    # ── 3. Nav2 with RRT* planner (starts after 14s, once /map exists) ─
    nav2 = TimerAction(
        period=14.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, "launch", "navigation2.launch.py")
                ),
                launch_arguments=[
                    ("use_sim_time", "True"),
                    ("params_file", nav2_params_path),
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
            gazebo,
            slam,
            nav2,
        ]
    )
