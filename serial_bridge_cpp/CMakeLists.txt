cmake_minimum_required(VERSION 3.8)
project(serial_bridge_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Node as a component (can be loaded into a component container)
add_library(serial_bridge_component SHARED
  src/serial_port.cpp
  src/serial_bridge_node.cpp
)
target_include_directories(serial_bridge_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(serial_bridge_component
  rclcpp rclcpp_components std_msgs geometry_msgs nav_msgs tf2_ros
)
# Also generates a standalone executable "serial_bridge_node"
rclcpp_components_register_node(serial_bridge_component
  PLUGIN "serial_bridge_cpp::SerialBridgeNode"
  EXECUTABLE serial_bridge_node
)

install(TARGETS serial_bridge_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_parser test/test_frame_parser.cpp)
  target_include_directories(test_frame_parser PRIVATE include)
endif()

ament_package()
