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
find_package(Boost 1.45.0 COMPONENTS filesystem)

# add_subdirectory(libs/fatrop)
add_subdirectory(libs/a-star)

install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME})

set(DEPENDENCIES
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
)

add_executable(state_estimate src/state_estimate.cpp)
ament_target_dependencies(state_estimate ${DEPENDENCIES})

add_executable(pid_pose_node src/pid_pose_node.cpp)
ament_target_dependencies(pid_pose_node ${DEPENDENCIES})

# add_executable(mpc_node src/mpc_node.cpp)
# target_include_directories(mpc_node PRIVATE deps/fatrop/fatrop ${Boost_INCLUDE_DIRS} )
# target_link_libraries(mpc_node fatrop ${Boost_LIBRARIES}) 
# ament_target_dependencies(mpc_node ${DEPENDENCIES})

add_executable(a_star_node src/a_star_node.cpp)
target_link_libraries(a_star_node a-star) 
target_include_directories(a_star_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/a-star/source)
ament_target_dependencies(a_star_node ${DEPENDENCIES})

add_executable(line_follower_path_publisher_node src/line_follower_path_publisher_node.cpp)
ament_target_dependencies(line_follower_path_publisher_node ${DEPENDENCIES})

add_executable(obstacle_publisher src/obstacle_publisher.cpp)
ament_target_dependencies(obstacle_publisher ${DEPENDENCIES})

add_executable(path_interpolation_node src/path_interpolation_node.cpp)
ament_target_dependencies(path_interpolation_node ${DEPENDENCIES})

# Install Python executables
install(PROGRAMS
scripts/path_publisher_node.py
scripts/pzb_teleop_node.py
scripts/pzb_teleop_low_level_node.py
DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  state_estimate
  pid_pose_node
  # mpc_node
  a_star_node
  obstacle_publisher
  path_interpolation_node
  line_follower_path_publisher_node
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
