#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 패키지 경로
    loc_pkg_dir = get_package_share_directory('localization')
    sim_pkg_dir = get_package_share_directory('go1_simulation')

    # hospital map
    map_file = os.path.join(sim_pkg_dir, 'maps', 'hospital.yaml')

    # launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='1.0')
    yaw = LaunchConfiguration('yaw', default='0.0')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='1.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # 1. Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_file}
            ]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),

        # 2. Odom localizer
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_pkg_dir, 'launch', 'odom_localizer.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # 3. Global localizer
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_pkg_dir, 'launch', 'global_localizer.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'x': x,
                'y': y,
                'yaw': yaw
            }.items(),
        ),

        # 4. ✅ Pose publisher (METHOD B)
        Node(
            package='localization',
            executable='go1_pose_publisher.py',
            name='go1_pose_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'map_frame': 'map'},
                {'base_frame': 'base'},
                {'publish_rate_hz': 20.0},
            ]
        ),
    ])

