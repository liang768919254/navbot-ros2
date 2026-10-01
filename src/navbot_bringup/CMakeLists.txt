cmake_minimum_required(VERSION 3.8)
project(navbot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ★ 把 launch / config / rviz / worlds / maps 目录装到 share 下
#   maps 里现在只有说明版 yaml，navbot_room.pgm 等你建图后由 map_saver 生成
install(
  DIRECTORY launch config rviz worlds maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
