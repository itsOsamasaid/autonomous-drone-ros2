cmake_minimum_required(VERSION 3.8)
project(drone_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  drone_navigation/vision_pose_bridge.py
  drone_navigation/sim_rangefinder_bridge.py
  drone_navigation/guided_hold_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  worlds
  models
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
