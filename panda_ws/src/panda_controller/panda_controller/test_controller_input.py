#!/usr/bin/env python3
#First find the correct device name for your controller using `ls /dev/input/` and look for something like `js0`. 
#Then run this node to see the raw joystick data being published on the `joy` topic. 
#You can use this information to map the axes and buttons to specific controls in your main teleoperation node.
#ros2 run joy joy_node --ros-args -p device:=/dev/input/js0

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyTester(Node):
    def __init__(self):
        super().__init__('joy_tester')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.get_logger().info('Joy tester started. Move your controller!')

    def joy_callback(self, msg):
        # Print all joystick data
        self.get_logger().info(f'Axes: {msg.axes}')
        self.get_logger().info(f'Buttons: {msg.buttons}')
        
        # Interpret specific controls based on your controller
        if len(msg.axes) >= 4:
            left_x = msg.axes[0]   # Left stick X
            left_y = msg.axes[1]   # Left stick Y  
            right_x = msg.axes[2]  # Right stick X (or triggers)
            right_y = msg.axes[3]  # Right stick Y
            
            self.get_logger().info(f'Left stick: ({left_x:.2f}, {left_y:.2f})')
            self.get_logger().info(f'Right stick: ({right_x:.2f}, {right_y:.2f})')

def main():
    rclpy.init()
    joy_tester = JoyTester()
    rclpy.spin(joy_tester)
    joy_tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()