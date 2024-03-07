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
- `launch_path_gen.launch.py` - launches the path generation node (standalone) and rviz to visualize generated paths

## Running the path generation node

The path generation can work as standalone, you can just spin up the node using the provided launchfile: `ros2 launch minimec_control launch_path_gen.launch.py`

You can then call the service with this sample input:

- To generate a spline path: `ros2 service call /path_generator/generate_plan minimec_msgs/srv/PlanRequest "{points: [{x: 0.0, y: 0.0},{x: 1.0, y: 0.0}, {x: 1.0, y: 1.0}, {x: 0.0, y: 1.0}, {x: 0.0, y: 2.0}, {x: 1.0, y: 2.0},{x: 1.0, y: 3.0}], frame_id: odom}"`

- To generate a circular path: `ros2 service call /generate_circular_plan minimec_msgs/srv/CircularPlanRequest "{frame_id: map, center: {x: 0.0, y: 0.0}, resolution: 0.01, radius: 2.0}"`

This will display the points in rviz and the generated path.

