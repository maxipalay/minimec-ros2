# Collection of C++ libraries and ROS2 packages to control the minimec


# Setup

- Follow the steps in `/setup` to cofigure the robot.
- To use the ros packages, compile the workspace. `colcon build --cmake-args -DBUILD_TESTING=OFF`, the `-DBUILD_TESTING` flag is optional, but it prevents from compiling tests. Note: cross compilation is recommended, as compilation on the Pi can take quite a bit.