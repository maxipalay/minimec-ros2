# minimec_lights

ROS2 Package that interfaces with Neopixel LEDs on the minimec. Node was tested to work on a Raspberry Pi 4.

## behavior

The `lights` node subscribes to `/cmd_vel`, and maps velocity commands to LED behaviors.

- When velocity commands are zero, the LEDs do a chase of rainbow colors.
- When velocity commands include x or y velocity, the direction of the linear speed is calculated and the LEDs light up on that section of the robot.
- When velocity commands are purely rotational the LEDs do a chase on the direction of rotation.
