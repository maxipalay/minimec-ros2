import board
import neopixel
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import math
import time

def wheel(pos):
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
    if pos < 0 or pos > 255:
        r = g = b = 0
    elif pos < 85:
        r = int(pos * 3)
        g = int(255 - pos * 3)
        b = 0
    elif pos < 170:
        pos -= 85
        r = int(255 - pos * 3)
        g = 0
        b = int(pos * 3)
    else:
        pos -= 170
        r = 0
        g = int(pos * 3)
        b = int(255 - pos * 3)
    return (r/2, g/2, b/2)

class Lights(Node):

    def __init__(self, ):

        super().__init__('minimec_lights')
        
        self.sub_cmd_vel = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        
        self.num_pixels = 142

        self.pixels = neopixel.NeoPixel(board.D10, self.num_pixels, auto_write=False)
        
        self.timer_control = self.create_timer(0.02, self.cb_timer)
        self.startup = True

        self.j = 0
        self.i = 0

        self.mode = 0

        self.chase_index = 0
    
    def cb_timer(self,):
        if self.mode == 0:
            self.rainbow_cycle()  # rainbow cycle with 1ms delay per step
        elif self.mode == 1:
            self.chase(True)
        elif self.mode == 2:
            self.chase(False)

    def cmd_vel_cb(self, msg):
        self.get_logger().error("received cmd_vel message")
        if msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0:
            self.mode = 0
        elif msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z > 0.0:
            self.mode = 1
        elif msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z < 0.0:
            self.mode = 2
        else:
            self.pixels.fill((0,0,0))
            self.mode = 3
            orientation = math.atan2(msg.linear.y, msg.linear.x)
            direction_index = self.angle_to_index(orientation)
            self.pixels[direction_index] = (255,255,255)
            self.get_logger().error(f"direction index: {direction_index}")
            for i in range(1,25):
                self.pixels[(direction_index+i)%(self.num_pixels-1)] = (int(255.0*(1.0-float(i)/24.0)),255,255)
                self.pixels[(direction_index-i)%(self.num_pixels-1)] = (int(255.0*(1.0-float(i)/24.0)),255,255)
            self.pixels.show()

        return

    def rainbow_cycle(self):
        pixel_index = (self.i * 256 // self.num_pixels) + self.j
        self.pixels[self.i] = wheel(pixel_index & 255)
        self.pixels.show()
        self.j+=1
        self.i+=1
        if self.j == 255:
            self.j=0
        if self.i == self.num_pixels:
            self.i = 0
        return
    
    def chase(self, dir):
        index = self.chase_index
        self.pixels.fill((0,0,0))
        self.pixels[index] = (255,255,255)
        for i in range(1,25):
            self.pixels[(index+i)%(self.num_pixels-1)] = (int(255.0*(1.0-float(i)/24.0)),255,255)
            self.pixels[(index-i)%(self.num_pixels-1)] = (int(255.0*(1.0-float(i)/24.0)),255,255)
        self.pixels.show()
        if dir:
            self.chase_index -= 1
            if self.chase_index < 0:
                self.chase_index += self.num_pixels
        else:
            self.chase_index += 1
            self.chase_index = self.chase_index % (self.num_pixels - 1)

    def angle_to_index(self, angle):
        # take the angle to 0,2pi
        angle = angle + math.pi
        # offset pi/6
        angle = angle - math.pi * (1.0/6.0)
        
        led_index = int(-angle * self.num_pixels / (2.0*math.pi)) % 142

        return abs(led_index)

    
def main(args=None):
    rclpy.init(args=args)
    l = Lights()
    rclpy.spin(l)
    rclpy.shutdown()


if __name__ == '__main__':
    main()