cmake_minimum_required(VERSION 3.8)
project(pzb_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# OpenCV
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
# Custom messages
find_package(pzb_msgs REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})


add_executable(camera_view src/camera_view.cpp)
ament_target_dependencies(camera_view rclcpp std_msgs OpenCV sensor_msgs cv_bridge pzb_msgs)

add_executable(decode_img src/decode_img.cpp)
ament_target_dependencies(decode_img rclcpp std_msgs OpenCV sensor_msgs cv_bridge pzb_msgs)

add_executable(line_detection src/line_detection.cpp)
ament_target_dependencies(line_detection rclcpp std_msgs OpenCV sensor_msgs cv_bridge pzb_msgs)

add_executable(dotted_detection src/dotted_detection.cpp)
ament_target_dependencies(dotted_detection rclcpp std_msgs OpenCV sensor_msgs cv_bridge pzb_msgs)

install(TARGETS
    camera_view
    line_detection
    decode_img
    dotted_detection
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
