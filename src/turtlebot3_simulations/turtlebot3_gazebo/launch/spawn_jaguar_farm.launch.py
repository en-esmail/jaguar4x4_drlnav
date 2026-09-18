from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

from launch.actions import TimerAction


from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    farm_world = os.path.join(
        get_package_share_directory('turtlebot3_drl'),
        'farm_environment',
        'worlds',
        'merged_farm.world'
    )


    # Jaguar description
    urdf_file = PathJoinSubstitution([
        FindPackageShare('jaguar_description'),
        'urdf',
        'jaguar.urdf.xacro'
    ])
    robot_description = Command(['xacro ', urdf_file])

    # Gazebo server + client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': farm_world}.items(),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Spawn Jaguar
    spawn_jaguar = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'jaguar', '-topic', 'robot_description', "-x", "0.0", "-y", "0.0", "-z", "0.1"],
        output='screen'
    )

    delayed_spawn = TimerAction(period=5.0, actions=[spawn_jaguar])


    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        delayed_spawn
    ])
