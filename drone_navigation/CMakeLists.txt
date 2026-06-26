cmake_minimum_required(VERSION 3.8)
project(drone_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install ALL Python nodes as executables
install(PROGRAMS
  drone_navigation/odom_tf_node.py
  drone_navigation/takeoff_node.py
  drone_navigation/cmd_vel_mux_node.py
  drone_navigation/altitude_hold_node.py
  drone_navigation/position_hold_node.py
  drone_navigation/mission_node.py
  drone_navigation/vision_pose_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install all resource directories
install(DIRECTORY
  launch
  config
  maps
  worlds
  models
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()