cmake_minimum_required(VERSION 3.8)
project(minimec_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(minimeclib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(odrive_can REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(velocity_parser src/velocity_parser.cpp)
ament_target_dependencies(velocity_parser rclcpp geometry_msgs odrive_can sensor_msgs)
target_link_libraries(velocity_parser minimeclib::minimeclib)

add_executable(odometry src/odometry.cpp)
ament_target_dependencies(odometry rclcpp geometry_msgs odrive_can sensor_msgs tf2 tf2_ros nav_msgs tf2_geometry_msgs)
target_link_libraries(odometry minimeclib::minimeclib)

install(TARGETS
  velocity_parser
  odometry
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
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
