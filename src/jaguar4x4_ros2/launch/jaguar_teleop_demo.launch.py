import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('jaguar4x4_ros2')
    
    # Path to RViz file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'jaguar_teleop_demo.rviz')

    ekf_config = os.path.join(
        get_package_share_directory('jaguar4x4_ros2'),
        'params',
        'ekf.yaml'
    )

    return LaunchDescription([
        # ---------------------------------------------------------
        # 1. Joystick Driver (joy_node)
        #    Reads raw data from the Logitech F710
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
        # 2. Teleop Twist Joy (Converts Joy -> Twist)
        #    Converts raw buttons to Geometry Twist messages
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
                'enable_button': 4,             # LB or RB usually
                'enable_turbo_button': 5,
            }],
            remappings=[
                ('/cmd_vel', '/ros_drrobot_motor_cmd'),
                ('/joy', '/joy')
            ]
        ),

        # ---------------------------------------------------------
        # 3. Jaguar Main Driver (drrobot_player)
        #    Talks to the robot hardware via TCP/IP
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
                'RobotBaseIP': '192.168.1.60',  # CHECK THIS IP!
                'RobotPortNum': 10001,
                'RobotSerialPort': '/dev/ttyS0',
                'MotorDir': 1,
                'WheelRadius': 0.135, # 0.135
                'WheelDistance': 0.35, # 0.52
                'MinSpeed': 0.1,
                'MaxSpeed': 1.0,
                'EncoderCircleCnt': 300,
            }]
        ),

        # ---------------------------------------------------------
        # 4. Sensor Converter Node
        #    Converts custom robot data to standard ROS Odometry/IMU
        # ---------------------------------------------------------
        Node(
            package='jaguar4x4_ros2',
            executable='drrobot_ros_sensor',
            name='drrobot_ros_sensor_node',
            output='screen'
        ),

        # # ---------------------------------------------------------
        # # 5. Rplidar Node
        # # ---------------------------------------------------------
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),

        # ---------------------------------------------------------
        # 6. RF2O Laser Odometry (NEW ADDITION)
        #    Calculates odometry from LiDAR scans
        # ---------------------------------------------------------
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',     # Must match LDS Lidar topic
                'odom_topic': '/odom_rf2o',      # Unique topic for EKF to consume
                'publish_tf': False,             # FALSE because EKF handles the TF
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 10.0                     # Hz
            }],
        ),
        

        # # TF Node: The NEW Static TF Publisher Node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf_pub',
            arguments=[
                # X =0.27, Y=0.05, Z (in meters)
                '0.27', '0.0', '0.40',
                
                # Yaw, Pitch, Roll (in radians)
                '0.0', '0.0', '0.0',
                
                # Parent Frame, Child Frame
                'base_link', 'laser'
            ],
        ),

        # ---------------------------------------------------------
        # 6. RViz2
        #    Visualizes the robot data
        # ---------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                ekf_config,
                {"use_sim_time": False},
            ],
        ),

    ])
