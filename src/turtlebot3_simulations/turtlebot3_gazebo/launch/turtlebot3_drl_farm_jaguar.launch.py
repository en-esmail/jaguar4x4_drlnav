from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
from launch.actions import TimerAction
from launch.substitutions import Command


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    pause = LaunchConfiguration('pause', default='true')
    # Packages
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    tb3_drl_pkg = get_package_share_directory('turtlebot3_drl')
    jquar_pkg = get_package_share_directory('jaguar4x4_ros2')

    # Paths
    world_file = os.path.join(tb3_drl_pkg, "farm_environment", "worlds", "merged_farm.world")

    xacro_file = os.path.join(jquar_pkg, 'urdf', 'jaguar.urdf.xacro')
    robot_desc = Command(['xacro ', xacro_file])


    # Export model path for Gazebo
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(tb3_drl_pkg, "farm_environment", "models") \
                                      + ":" + os.environ.get("GAZEBO_MODEL_PATH", "")

    # Arguments
    declare_world_cmd = DeclareLaunchArgument(
        "world", default_value=world_file,
        description="World file to load"
    )

    # # Gazebo (server + client)
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(gazebo_ros_pkg, "launch", "gazebo.launch.py")
    #     ),
    #     launch_arguments={
    #         "world": LaunchConfiguration("world"),
    #     }.items(),
    # )

    tf_spawn= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc, 
            'use_sim_time': True
        }]
    )
        # ---------------------------------------------------------
        # 3. Spawn the Robot in Gazebo
        #    It listens to the 'robot_description' topic from the node above
        # ---------------------------------------------------------
    jq_spawn= Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'jaguar', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file, 'pause' : pause}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
            ),
        ),
        declare_world_cmd,
        # gazebo_launch,
        tf_spawn,
        jq_spawn
    ])
