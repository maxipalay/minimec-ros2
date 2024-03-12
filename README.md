# minimec-ros2

The minimec is a custom mecanum wheel base made for development. This repository contains a simple C++ library and ROS2 packages that make the software running on the robot.

Although this was purposely done for the Minimec, I tried to keep the software decoupled from the hardware. A user could ideally swap out the `minimec_driver` package for one that implements communication with their own hardware and still have mostly everything working.

## ROS2 architecture

[diagram]

### Packages
- `minimec_bringup` - necessary launchfiles and configuration to get the minimec up and running quickly.
- `minimec_control` - all things controls. Kinematics, odometry, path generation, trajectory tracking.
- `minimec_description` - robot definition, URDF, meshes.
- `minimec_driver` - interface with motion hardware (odrives). Although the odrive-provided nodes are used to communicate with the odrives over CAN, this provides an extra layer of abstraction.
- `minimec_msgs`- custom messages and services for minimec operation.
- `minimec_lights` - control of Neopixel LEDs mounted on the minimec.

Each package contains a README that goes into more detail.

## minimeclib - a C++ simple kinematics library for mecanum wheel robots

The `MecanumDrive` class is the core of this library.

It provides the following functions:
- FKin
- IKin
- setWheelOffsets

## setup

A `/setup` folder is included in this repository to help set up the robot easily and quickly. This is already done, but can serve as reference if there is a need to reflash the robot's storage.

- Follow the steps in `/setup` to cofigure the robot.

## building

The same workspace should be on the robot and your computer.

Compiling for the robot:

`colcon build --cmake-args -DBUILD_TESTING=OFF -DBUILD_ROBOT=ON`, the `-DBUILD_TESTING` flag is optional, but it prevents from compiling tests. Note: cross compilation is recommended, as compilation on the Pi can take quite a bit.

Compiling for the control computer:

`colcon build --cmake-args -DBUILD_TESTING=OFF -DBUILD_ROBOT=OFF`

## Launching the minimec

Make sure the robot and your computer are on the same network, and share the same `ROS_DOMAIN_ID`.

On the robot:

`ros2 launch minimec_bringup launch_minimec.launch.py cmd_src:=teleop` - this will launch the necessary nodes on the robot, and the robot just listens to the topic `/cmd_vel` and moves accordingly (no feedback). You can use the `teleop_twist_keyboard` node to drive the robot around.

For launching the robot to track paths, you should run:
`ros2 launch minimec_bringup launch_minimec.launch.py --ros-args cmd_src:=path` - this will additionally launch the `trajectory_tracking` node. Whenever a path is published, the tracking node will start following.

Note: be mindful of the current robot location when you run the trajectory tracking. If the robot is too far from the first point in the path, it will try to make the correction pretty quickly.

On the computer:

`ros2 launch minimec_bringup launch_command.launch.py` - this will launch rviz and the path generation node.