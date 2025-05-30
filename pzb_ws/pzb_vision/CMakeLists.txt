cmake_minimum_required(VERSION 3.8)
project(pzb_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pzb_msgs REQUIRED)

set(DEPENDENCIES
  ament_cmake
  urdf
  robot_state_publisher
  xacro
  geometry_msgs
  visualization_msgs
  std_msgs
  std_srvs
  rclcpp
  sensor_msgs
  tf2
  nav_msgs
  tf2_ros
  Eigen3
  tf2_geometry_msgs
  pzb_msgs
  OpenCV
  cv_bridge
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

add_executable(camera_view src/camera_view.cpp)
ament_target_dependencies(camera_view ${DEPENDENCIES})

add_executable(decode_img src/decode_img.cpp)
ament_target_dependencies(decode_img ${DEPENDENCIES})

add_executable(line_detection src/line_detection.cpp)
ament_target_dependencies(line_detection ${DEPENDENCIES})

add_executable(dotted_detection src/dotted_detection.cpp)
ament_target_dependencies(dotted_detection ${DEPENDENCIES})

add_executable(alignment_node src/alignment_node.cpp)
ament_target_dependencies(alignment_node ${DEPENDENCIES})

install(TARGETS
    camera_view
    line_detection
    decode_img
    dotted_detection
    alignment_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python executables

install(DIRECTORY
    scripts
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/signal_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

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
