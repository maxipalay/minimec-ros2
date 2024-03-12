# minimec_control

ROS2 package for controlling a mecanum wheel base. Made for controlling the Minimec, but can be extended for other platforms.

## Nodes

- `kinematics`: node that implements forward kinematics, taking an input twist and outputting wheel commands.
- `odometry`: node that implements odometry - estimation of position according to encoder feedback. This takes in joint states and publishes the odometry transform to a topic and broadcasts it to the tf tree.
- `path_generator`: node that generates spline and circular paths.
- `trajectory_tracking`: node that publisher velocity commands to track a generated path using feedforward + PI feedback control

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

- To generate a spline path with fixed orienation: `ros2 service call /generate_spline_plan minimec_msgs/srv/SplinePlanRequest "{points: [{x: 0.0, y: 0.0},{x: 1.0, y: 0.0}, {x: 1.0, y: 1.0}, {x: 2.0, y: 1.0}, {x: 2.0, y: 0.0}, {x: 3.0, y: 0.0},{x: 3.0, y: 1.0},{x: 4.0, y: 1.0},{x: 4.0, y: 0.0},{x: 5.0, y: 0.0}], heading_mode: fixed, frame_id: odom, heading_angle: 0.0, resolution: 0.0075}""`

- To generate a spline path with point orientation: `ros2 service call /generate_spline_plan minimec_msgs/srv/SplinePlanRequest "{points: [{x: 0.0, y: 0.0},{x: 1.0, y: 0.0}, {x: 1.0, y: 1.0}, {x: 2.0, y: 1.0}, {x: 2.0, y: 0.0}, {x: 3.0, y: 0.0},{x: 3.0, y: 1.0},{x: 4.0, y: 1.0},{x: 4.0, y: 0.0},{x: 5.0, y: 0.0}], heading_mode: point, frame_id: odom, heading_point: {x: 6.0, y: 0.0}, resolution: 0.0075}"`

- To generate a circular path: `ros2 service call /generate_circular_plan minimec_msgs/srv/CircularPlanRequest "{frame_id: odom, center: {x: 0.75, y: 0.0}, resolution: 0.005, radius: 0.75}"`

This will display the points in rviz and the generated path.

