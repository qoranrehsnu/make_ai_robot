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
    pkg_perc = get_package_share_directory('perception')
    pkg_test = get_package_share_directory('module_test')

    # --- 2. Define The Launch Files ---
    #Simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sim, 'launch', 'go1.gazebo.launch.py')),
        launch_arguments={'use_gt_pose': 'false'}.items()
    )

    #Localization (Odom + Global)
    main_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_loc, 'launch', 'main_localization.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    #Robot Control
    control_node = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 
            'ros2', 'run', 'unitree_guide2', 'junior_ctrl',
            '--ros-args', '-p', 'use_sim_time:=true'
        ],
        output='screen'
    )
    #Planner & Tracker
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

    #Perception & Viewer
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_perc, 'launch', 'perception_launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    viewer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_test, 'launch', 'interface_viewer.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # --- 3. Create the Sequence  ---
    ld = LaunchDescription()

    # 0s: Start Sim and Map
    ld.add_action(sim_launch)

    #Start Control (Pop-up window appears)
    ld.add_action(TimerAction(period=5.0, actions=[control_node]))

    #Start Localization
    ld.add_action(TimerAction(period=40.0, actions=[main_loc_launch]))

    #Start Planning & Tracking Stack
    ld.add_action(TimerAction(period=45.0, actions=[planner_launch, path_tracker_launch]))

    #Start Perception & viewer 
    ld.add_action(TimerAction(period=47.0, actions=[perception_launch]))
    ld.add_action(TimerAction(period=50.0, actions=[viewer_launch]))
    return ld