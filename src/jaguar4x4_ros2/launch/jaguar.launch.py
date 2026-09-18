import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command  # <--- IMPORT COMMAND
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('jaguar4x4_ros2')
    
    # Path to RViz file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'jaguar_teleop_demo.rviz')

    # Path to EKF params
    ekf_config = os.path.join(pkg_share, 'params', 'ekf.yaml')

    # ---------------------------------------------------------
    # NEW: Path to URDF/Xacro file
    # (Copied from your Gazebo file logic)
    # ---------------------------------------------------------
    xacro_file = os.path.join(pkg_share, 'urdf', 'jaguar.urdf.xacro')

    # ---------------------------------------------------------
    # NEW: Process the URDF
    # ---------------------------------------------------------
    robot_desc = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # ---------------------------------------------------------
        # 1. Joystick Driver (joy_node)
        # ---------------------------------------------------------
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/by-id/usb-Logitech_Wireless_Gamepad_F710_6E7C9908-joystick',
                'deadzone': 0.01,
                'autorepeat_rate': 20.0,
            }]
        ),

        # ---------------------------------------------------------
        # 2. Teleop Twist Joy
        # ---------------------------------------------------------
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[{
                'axis_linear.x': 1,             
                'scale_linear.x': 200.0,        
                'scale_linear_turbo.x': 600.0,  
                'axis_angular.yaw': 3,          
                'scale_angular.yaw': 200.0,     
                'enable_button': 4,
                'enable_turbo_button': 5,
            }],
            remappings=[
                ('/cmd_vel', '/ros_drrobot_motor_cmd'),
                ('/joy', '/joy')
            ]
        ),

        # ---------------------------------------------------------
        # 3. Jaguar Main Driver (drrobot_player)
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
                'RobotBaseIP': '192.168.1.60',
                'RobotPortNum': 10001,
                'RobotSerialPort': '/dev/ttyS0',
                'MotorDir': 1,
                'WheelRadius': 0.135,
                'WheelDistance': 0.35,
                'MinSpeed': 0.1,
                'MaxSpeed': 1.0,
                'EncoderCircleCnt': 300,
            }]
        ),

        # ---------------------------------------------------------
        # 4. Sensor Converter Node
        # ---------------------------------------------------------
        Node(
            package='jaguar4x4_ros2',
            executable='drrobot_ros_sensor',
            name='drrobot_ros_sensor_node',
            output='screen'
        ),

        # ---------------------------------------------------------
        # NEW NODE: Robot State Publisher
        # This publishes the URDF TFs and allows RViz to see the model
        # ---------------------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc, 
                'use_sim_time': False # Real world is False
            }]
        ),

        # ---------------------------------------------------------
        # NEW NODE: Joint State Publisher
        # Keeps the wheels/joints from collapsing if the hardware driver 
        # doesn't publish /joint_states.
        # ---------------------------------------------------------
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui': False}]
        ),

        # ---------------------------------------------------------
        # 5. LDS LiDAR Node
        # ---------------------------------------------------------
        Node(
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='hlds_laser_publisher',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'frame_id': 'laser' 
            }]
        ),

        # ---------------------------------------------------------
        # 6. RF2O Laser Odometry
        # ---------------------------------------------------------
        # Node(
        #     package='rf2o_laser_odometry',
        #     executable='rf2o_laser_odometry_node',
        #     name='rf2o_laser_odometry',
        #     output='screen',
        #     parameters=[{
        #         'laser_scan_topic': '/scan',
        #         'odom_topic': '/odom_rf2o',
        #         'publish_tf': False,
        #         'base_frame_id': 'base_link',
        #         'odom_frame_id': 'odom',
        #         'init_pose_from_topic': '',
        #         'freq': 10.0
        #     }],
        # ),

        # ---------------------------------------------------------
        # 7. TF Node: Static TF Publisher (REMOVED/COMMENTED)
        # Because 'robot_state_publisher' now reads the URDF 
        # and publishes the 'base_link' -> 'laser' transform automatically.
        # Uncomment only if your URDF does NOT include the laser frame.
        # ---------------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf_pub',
            arguments=['0.0', '0.0', '0.44', '0.0', '0.0', '0.0', 'base_link', 'laser'],
        ),

        # ---------------------------------------------------------
        # 8. RViz2
        # ---------------------------------------------------------
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file],
        #     output='screen'
        # ),

        # ---------------------------------------------------------
        # 9. EKF Filter
        # ---------------------------------------------------------
    #     Node(
    #         package="robot_localization",
    #         executable="ekf_node",
    #         name="ekf_filter_node",
    #         output="screen",
    #         parameters=[
    #             ekf_config,
    #             {"use_sim_time": False},
    #         ],
    #     ),
    ])