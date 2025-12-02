#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Launch arguments
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    declare_x = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Initial guess x position for PF'
    )
    declare_y = DeclareLaunchArgument(
        'y',
        default_value='1.0',
        description='Initial guess y position for PF'
    )
    declare_yaw = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial guess yaw for PF'
    )

    # EKF odometry node (IMU + ICP)
    ekf_odom = Node(
        package='localization',
        executable='ekf_odom_localizer_node.py',
        name='ekf_odom_localizer',
        output='screen',
        parameters=[
            {
                'base_frame': 'base',
                'odom_frame': 'odom',
                'scan_topic': '/scan',
                'imu_topic': '/imu/data',
            }
        ]
    )

    # Particle Filter Global Localizer node
    pf_global = Node(
        package='localization',
        executable='pf_global_localizer_node.py',
        name='pf_global_localizer',
        output='screen',
        parameters=[
            {
                'map_topic': '/map',
                'scan_topic': '/scan',
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base',
                'num_particles': 200,
                'init_x': x,
                'init_y': y,
                'init_yaw': yaw
            }
        ]
    )

    return LaunchDescription([
        declare_x,
        declare_y,
        declare_yaw,
        ekf_odom,
        pf_global
    ])

