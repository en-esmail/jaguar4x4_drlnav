import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'jaguar4x4_ros2'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Path to the XACRO file
    xacro_file = os.path.join(pkg_share, 'urdf', 'jaguar.urdf.xacro')
    
    # 2. Path to RViz file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'jaguar_teleop_demo.rviz')

    # 3. Use Command to process the xacro file into a URDF string
    # This converts the macros and variables (like ${M_PI}) into plain XML
    robot_desc = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # ---------------------------------------------------------
        # 1. Start Gazebo Environment
        # ---------------------------------------------------------
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # ---------------------------------------------------------
        # 2. Publish Robot State (TF)
        #    This node publishes the transforms and the 'robot_description' topic
        # ---------------------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc, 
                'use_sim_time': True
            }]
        ),

        # ---------------------------------------------------------
        # 3. Spawn the Robot in Gazebo
        #    It listens to the 'robot_description' topic from the node above
        # ---------------------------------------------------------
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'jaguar', '-topic', 'robot_description'],
            output='screen'
        ),

        # ---------------------------------------------------------
        # 4. RViz2 (Optional)
        #    Useful to compare the Simulation vs the ROS TFs
        # ---------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
