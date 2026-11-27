cmake_minimum_required(VERSION 3.8)
project(r8_aruco_detector)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  r8_aruco_detector/aruco_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aruco_node
)

ament_package()
