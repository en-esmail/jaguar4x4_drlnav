from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

from launch.actions import TimerAction

def generate_launch_description():
    # Packages
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    tb3_drl_pkg = get_package_share_directory('turtlebot3_drl')

    # Paths
    world_file = os.path.join(tb3_drl_pkg, "farm_environment", "worlds", "merged_farm.world")
    model_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        "models",
        "turtlebot3_burger",
        "model.sdf"
    )

    # Export model path for Gazebo
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(tb3_drl_pkg, "farm_environment", "models") \
                                      + ":" + os.environ.get("GAZEBO_MODEL_PATH", "")

    # Arguments
    declare_world_cmd = DeclareLaunchArgument(
        "world", default_value=world_file,
        description="World file to load"
    )

    # Gazebo (server + client)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
        }.items(),
    )

    # Spawn TurtleBot3
    spawn_tb3 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "turtlebot3",
            "-file", model_file,
            "-x", "0.0", "-y", "0.0", "-z", "0.1"
        ],
        output="screen"
    )

    delayed_spawn = TimerAction(period=5.0, actions=[spawn_tb3])

    return LaunchDescription([
        declare_world_cmd,
        gazebo_launch,
        delayed_spawn
    ])
