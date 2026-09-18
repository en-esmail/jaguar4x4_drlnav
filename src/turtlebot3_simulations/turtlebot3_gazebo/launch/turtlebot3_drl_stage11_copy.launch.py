#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch.actions import TimerAction
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pause = LaunchConfiguration('pause', default='true')

    # Use custom farm world instead of TB3 stage11
    tb3_drl_pkg = get_package_share_directory('turtlebot3_drl')
    world = os.path.join(
        tb3_drl_pkg,
        "farm_environment",
        "worlds",
        "merged_farm.world"
    )

    # Package directories
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Write current stage to tmp
    with open('/tmp/drlnav_current_stage.txt', 'w') as file:
        file.write("11\n")

    spawn_tb3 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "turtlebot3",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.1"
        ],
        output="screen"
    )

    delayed_spawn = TimerAction(period=5.0, actions=[spawn_tb3])

    return LaunchDescription([
        # Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world, 'pause': pause}.items(),
        ),

        # Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # Robot state publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        delayed_spawn,
    ])
