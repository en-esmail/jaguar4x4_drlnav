cmake_minimum_required(VERSION 3.8)
project(jaguar4x4_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================================
# 1. Find Dependencies
# ========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Sensor Node Dependencies
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# ========================================
# 2. Define Custom Messages
# ========================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GPSInfo.msg"
  "msg/IMUData.msg"
  "msg/MotorData.msg"
  "msg/MotorDataArray.msg"
  "msg/MotorBoardInfo.msg"
  "msg/MotorBoardInfoArray.msg"
  DEPENDENCIES std_msgs
)

# ========================================
# 3. Build the Driver Library
# ========================================
add_library(drrobot_driver SHARED
  src/DrRobotMotionSensorDriver.cpp
)
# Tells CMake to look in the 'include' folder for headers
target_include_directories(drrobot_driver PUBLIC include)

# Link message headers (using modern CMake target)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(drrobot_driver "${cpp_typesupport_target}")

# ========================================
# 4. Build the Keyboard Node
# ========================================
add_executable(drrobot_keyboard_teleop src/drrobot_keyboard_teleop.cpp)
ament_target_dependencies(drrobot_keyboard_teleop rclcpp std_msgs)
target_link_libraries(drrobot_keyboard_teleop "${cpp_typesupport_target}")

# ========================================
# 5. Build the Player Node (Main Driver)
# ========================================
add_executable(drrobot_player src/drrobot_player.cpp)
ament_target_dependencies(drrobot_player rclcpp std_msgs geometry_msgs nav_msgs)
target_include_directories(drrobot_player PUBLIC include)
target_link_libraries(drrobot_player drrobot_driver "${cpp_typesupport_target}")

# ========================================
# 6. Build the ROS Sensor Node
# ========================================
add_executable(drrobot_ros_sensor src/drrobot_ros_sensor.cpp)
ament_target_dependencies(drrobot_ros_sensor 
  rclcpp 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  sensor_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  tf2_eigen
)
target_link_libraries(drrobot_ros_sensor Eigen3::Eigen "${cpp_typesupport_target}")

# ========================================
# 7. Install Rules
# ========================================
install(TARGETS
  drrobot_driver
  drrobot_keyboard_teleop
  drrobot_player
  drrobot_ros_sensor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF and Meshes for Gazebo
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
