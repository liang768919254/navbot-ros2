cmake_minimum_required(VERSION 3.10)
project(navbot_chassis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(chassis_node src/chassis_node.cpp)
ament_target_dependencies(chassis_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS chassis_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
