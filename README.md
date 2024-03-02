# minimec-ros2 [wip]

The minimec is a custom mecanum wheel base made for development. This repository contains a simple C++ library and ROS2 packages that make the software running on the robot.

Although this was purposely done for the Minimec, I tried to keep the software decoupled from the hardware. A user could ideally swap out the `minimec_driver` package for one that implements communication with their own hardware and still have mostly everything working.

More documentation to be added soon.

## ROS2 architecture

[diagram]

### Packages
- `minimec_bringup`
- `minimec_control`
- `minimec_description`
- `minimec_driver`
- `minimec_msgs`

## minimeclib - a C++ simple kinematics library for mecanum wheel robots

The `MecanumDrive` class is the core of this library.

It provides the following functions:
- FKin
- IKin
- setWheelOffsets

## setup

A `/setup` folder is included in this repository to help set up the robot easily and quickly.

- Follow the steps in `/setup` to cofigure the robot.
- To use the ros packages, compile the workspace. `colcon build --cmake-args -DBUILD_TESTING=OFF`, the `-DBUILD_TESTING` flag is optional, but it prevents from compiling tests. Note: cross compilation is recommended, as compilation on the Pi can take quite a bit.