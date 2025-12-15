#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # =========================
    # Launch arguments
    # =========================
    declare_use_gt_pose_cmd = DeclareLaunchArgument(
        'use_gt_pose',
        default_value='true',
        description='Use ground truth pose from Gazebo'
    )

    use_gt_pose = LaunchConfiguration('use_gt_pose')

    # =========================m
    # Package directories
    # =========================
    go1_pkg_dir = get_package_share_directory('go1_siulation')
    path_tracker_pkg_dir = get_package_share_directory('path_tracker')

    # =========================
    # 1) Gazebo + GO1 bringup
    # =========================
    go1_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go1_pkg_dir, 'launch', 'go1.gazebo.launch.py')
        ),
        launch_arguments={'use_gt_pose': use_gt_pose}.items()
    )

    # =========================
    # 2) Map visualization
    # =========================
    visualize_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go1_pkg_dir, 'launch', 'visualize_map.launch.py')
        )
    )

    # =========================
    # 3) A* path planner
    # ros2 run astar_planner path_planner_node
    # =========================
    astar_planner_node = Node(
        package='astar_planner',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen'
    )

    # =========================
    # 4) Path tracker
    # ros2 launch path_tracker path_tracker_launch.py
    # =========================
    path_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(path_tracker_pkg_dir, 'launch', 'path_tracker_launch.py')
        )
    )

    # =========================
    # LaunchDescription
    # =========================
    ld = LaunchDescription()

    ld.add_action(declare_use_gt_pose_cmd)

    ld.add_action(TimerAction(period=0.0, actions=[go1_gazebo_launch]))
    ld.add_action(TimerAction(period=5.0, actions=[visualize_map_launch]))
    ld.add_action(TimerAction(period=10.0, actions=[astar_planner_node]))
    ld.add_action(TimerAction(period=12.0, actions=[path_tracker_launch]))

    return ld
