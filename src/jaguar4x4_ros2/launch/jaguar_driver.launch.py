import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('jaguar4x4_ros2')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'jaguar_teleop_demo.rviz')

    return LaunchDescription([
        # ---------------------------------------------------------
        # 1. Jaguar Main Driver (drrobot_player)
        # ---------------------------------------------------------
        Node(
            package='jaguar4x4_ros2',
            executable='drrobot_player',
            name='drrobot_player_node',
            output='screen',
            respawn=True,
            parameters=[{
                'RobotID': 'DrRobot',
                'RobotType': 'Jaguar',
                'RobotCommMethod': 'Network',
                'RobotBaseIP': '192.168.1.60',  # Ensure this matches your robot
                'RobotPortNum': 10001,
                'RobotSerialPort': '/dev/ttyS0',
                'MotorDir': 1,
                'WheelRadius': 0.135,
                'WheelDistance': 0.52,
                'MinSpeed': 0.1,
                'MaxSpeed': 1.0,
                'EncoderCircleCnt': 300,
            }]
        ),

        # ---------------------------------------------------------
        # 2. Sensor Converter Node
        # ---------------------------------------------------------
        Node(
            package='jaguar4x4_ros2',
            executable='drrobot_ros_sensor',
            name='drrobot_ros_sensor_node',
            output='screen'
        ),

        # ---------------------------------------------------------
        # 3. RViz2
        # ---------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
