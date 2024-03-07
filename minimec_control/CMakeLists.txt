cmake_minimum_required(VERSION 3.8)
project(minimec_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_ROBOT "Build for running on the robot." ON)

if (!BUILD_ROBOT)
    set(CMAKE_PREFIX_PATH "/usr/local/lib64/cmake/tinysplinecxx")
    find_package(tinysplinecxx REQUIRED)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Doxygen)
find_package(rclcpp REQUIRED)
find_package(minimeclib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(minimec_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Armadillo REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${ARMADILLO_INCLUDE_DIRS})

add_executable(kinematics src/kinematics.cpp)
ament_target_dependencies(kinematics rclcpp geometry_msgs minimec_msgs)
target_link_libraries(kinematics minimeclib::minimeclib)

add_executable(odometry src/odometry.cpp)
ament_target_dependencies(odometry rclcpp geometry_msgs sensor_msgs tf2 tf2_ros nav_msgs tf2_geometry_msgs)
target_link_libraries(odometry minimeclib::minimeclib)

add_executable(commands src/commands.cpp)
ament_target_dependencies(commands rclcpp geometry_msgs sensor_msgs tf2 tf2_ros nav_msgs tf2_geometry_msgs)
target_link_libraries(commands ${ARMADILLO_LIBRARIES})

if (!BUILD_ROBOT)
    add_executable(path_generator src/path_generator.cpp)
    ament_target_dependencies(path_generator rclcpp minimec_msgs visualization_msgs)
    target_link_libraries(path_generator ${TINYSPLINECXX_LIBRARIES})
endif()

install(TARGETS
  kinematics
  odometry
  commands
  DESTINATION lib/${PROJECT_NAME})

if (!BUILD_ROBOT)
  install(TARGETS
  path_generator
  DESTINATION lib/${PROJECT_NAME})
endif()

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

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

ament_package()
