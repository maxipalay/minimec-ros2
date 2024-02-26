# minimec_control

ROS2 package for controlling a mecanum wheel base. Made for controlling the Minimec, but can be extended for other platforms.

## Nodes

- `kinematics`: node that implements forward kinematics, taking an input twist and outputting wheel commands.
- `odometry`: node that implements odometry - estimation of position according to encoder feedback. This takes in joint states and publishes the odometry transform to a topic and broadcasts it to the tf tree.

## Configuration


A file for configuring basic parameters is included in `config/params.yaml`. Parameters include:
- wheel radius
- wheelbase
- wheel track

## Launch files

- `launch_control.launch.py` - launches the kinematics and odometry nodes, passing them the parameter file.