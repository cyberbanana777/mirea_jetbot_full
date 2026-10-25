cmake_minimum_required(VERSION 3.5)
project(realsense_d435i_configs)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
