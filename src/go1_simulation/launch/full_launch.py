import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1. Get Package Directories ---
    pkg_sim = get_package_share_directory('go1_simulation')
    pkg_loc = get_package_share_directory('localization')
    pkg_plan = get_package_share_directory('astar_planner')
    pkg_track = get_package_share_directory('path_tracker')

    # --- 2. Define The Launch Files ---
    # A. Simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sim, 'launch', 'go1.gazebo.launch.py')),
        launch_arguments={'use_gt_pose': 'false'}.items()
    )

    # B. Map Server (Load Static Map)
    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sim, 'launch', 'visualize_map.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # C. Localization (Odom + Global)
    odom_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_loc, 'launch', 'odom_localizer.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    global_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_loc, 'launch', 'global_localizer.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    #D. Robot Control
    control_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'unitree_guide2', 'junior_ctrl'],
        output='screen'
    )

    # E. Planner & Tracker
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_plan, 'launch', 'astar_planner.launch.py')),
        launch_arguments={
            'use_gazebo': 'true',
            'use_rviz': 'false', # Don't open a second RViz
            'use_sim_time': 'true'
        }.items()
    )

    path_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_track, 'launch', 'path_tracker_launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # --- 3. Create the Sequence  ---
    ld = LaunchDescription()

    # 0s: Start Sim and Map
    ld.add_action(sim_launch)
    ld.add_action(map_launch)

    # 5s: Start Control (Pop-up window appears)
    ld.add_action(TimerAction(period=5.0, actions=[control_node]))

    # 10s: Start Localization
    ld.add_action(TimerAction(period=10.0, actions=[odom_loc_launch, global_loc_launch]))

    # 15s: Start Planning & Tracking Stack
    ld.add_action(TimerAction(period=15.0, actions=[planner_launch, path_tracker_launch]))

    return ld