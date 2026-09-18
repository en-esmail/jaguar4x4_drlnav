from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pause = LaunchConfiguration('pause', default='true')

    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    tb3_drl_pkg = get_package_share_directory('turtlebot3_drl')
    jaguar_pkg = get_package_share_directory('jaguar4x4_ros2')

    world_file = os.path.join(
        tb3_drl_pkg, "farm_environment", "worlds", "merged_farm.world"
    )

    xacro_file = os.path.join(jaguar_pkg, 'urdf', 'jaguar.urdf.xacro')
    robot_desc = Command(['xacro ', xacro_file])

    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(
        tb3_drl_pkg, "farm_environment", "models"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    spawn_jaguar = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'jaguar', '-topic', 'robot_description', '-z', '5'],
        output='screen'
    )

    # jq_spawn = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', 'jaguar',
    #         '-topic', 'robot_description',
    #         '-x', '0.0',
    #         '-y', '0.0',
    #         '-z', '0.15',
    #         '-R', '0.0',
    #         '-P', '0.0',
    #         '-Y', '0.0'
    #     ],
    #     output='screen'
    # )

    return LaunchDescription([


        gazebo,
        robot_state_pub,
        TimerAction(
            period=3.0,
            actions=[spawn_jaguar]
        )
    ])
