import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'box_bot_description'
    
    # 1. Path to your URDF
    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', 'box_bot.urdf')

    # 2. Launch Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. Robot State Publisher
    # This node reads the URDF and transforms from /joint_states to position the wheels
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_path).read(),
            'use_sim_time': True
        }]
    )

    # 4. Spawn the Robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'box_bot', '-z', '0.1'],
        output='screen'
    )

    # 5. Bridge (The "Translator")
    # Added /model/box_bot/joint_state to bring wheel rotations to ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Format: /gz_topic@ros_msg_type@gz_msg_type
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Bridge the joint states
            '/model/box_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Bridge the TF (This connects the wheels in the PDF)
            '/model/box_bot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        remappings=[
            ('/model/box_bot/joint_state', '/joint_states'),
            ('/model/box_bot/tf', '/tf')
        ],
        output='screen'
    )

    # 6. Obstacle Avoider
    obstacle_avoider = Node(
        package='box_bot_description',
        executable='obstacle_avoider',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        obstacle_avoider,
    ])
#Test the gitignore file to make sure it's working.