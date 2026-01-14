import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'box_bot_description'
    
    # 1. Path Setup
    pkg_share = get_package_share_directory(package_name)
    map_file = os.path.join(pkg_share, 'maps', 'my_new_mapp.yaml')
    
    #  You will need a params file for the planner and controller to work properly.
    # If you don't have one yet, these nodes will use default settings.
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # 2. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params_path, {'use_sim_time': True}, 
                        {'yaml_filename': map_file}] 
        ),

        # 3. AMCL (Localization)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params_path, {'use_sim_time': True, 'transform_tolerance': 1.0}]
        ),

        # 4. Planner Server (The "Navigator") - Finds the path on the map
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_path, {'use_sim_time': True}]
        ),

        # 5. Controller Server (The "Driver") - Sends cmd_vel to the robot
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_path, {'use_sim_time': True}]
        ),

        # 5b. Behavior Server (Recoveries) - Required for bt_navigator
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_path, {'use_sim_time': True}]
        ),

        # 6. BT Navigator (The "Brain") - Coordinates the planner and controller
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_path, {'use_sim_time': True}]
        ),

        # 7. Lifecycle Manager (UPDATED to include all new nodes)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 
                                        'amcl', 
                                        'planner_server', 
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator']}]
        )
    ])