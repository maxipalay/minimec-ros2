# minimec_driver

ROS2 package for controlling a mecanum wheel base. This package interfaces with the hardware directly or with abstractions of it. Example: Odrive provides nodes that offer topics and services, this package uses those and translate the information to ros-integration-friendly topics/services as needed for the Minimec.

## Nodes

- `minimec_driver`: subscribes to topics offered by the odrive controllers and publishes to topics that control the motors.

## Configuration


A file for configuring basic parameters is included in `config/params.yaml`. Parameters include: names for the different topics the node should subscribe to. This file also serves as mapping between the axes of the robot and the odrives (relates odrive<->wheel in robot description).

## Launch files

- `launch_driver.launch.py` - launches the minimec_driver node and passes the parameters in the params file.