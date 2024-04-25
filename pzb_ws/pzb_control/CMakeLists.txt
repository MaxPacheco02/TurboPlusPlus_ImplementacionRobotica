cmake_minimum_required(VERSION 3.8)
project(pzb_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pzb_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME})

add_executable(state_estimate src/state_estimate.cpp)
ament_target_dependencies(
  state_estimate
  geometry_msgs
  visualization_msgs
  std_msgs
  std_srvs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  nav_msgs
  Eigen3
  tf2_geometry_msgs
  pzb_msgs
)

add_executable(guidance_node src/guidance_node.cpp)
ament_target_dependencies(
  guidance_node
  geometry_msgs
  visualization_msgs
  std_msgs
  rclcpp
  std_srvs
  sensor_msgs
  tf2
  nav_msgs
  tf2_ros
  Eigen3
  tf2_geometry_msgs
  pzb_msgs
)

# install(PROGRAMS
#   scripts/prisma.py
#   scripts/speech_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS
    state_estimate
    guidance_node
    DESTINATION lib/${PROJECT_NAME})

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

ament_package()
