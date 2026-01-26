#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class VelocityTester(Node):
    def __init__(self):
        super().__init__('velocity_tester')
        self.publisher = self.create_publisher(Twist, '/panda_velocity_cmd', 10)
        self.get_logger().info("Velocity tester started!")

    def send_velocity(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, 
                     angular_x=0.0, angular_y=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.x = angular_x
        msg.angular.y = angular_y
        msg.angular.z = angular_z
        self.publisher.publish(msg)
        self.get_logger().info(f"Sending: linear=[{linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}], angular=[{angular_x:.2f}, {angular_y:.2f}, {angular_z:.2f}]")

    def stop(self):
        self.send_velocity()
        self.get_logger().info("STOPPED")

def test_movement(tester, description, **kwargs):
    print(f"\n=== {description} ===")
    for i in range(20):  # 2 seconds at 10Hz
        tester.send_velocity(**kwargs)
        rclpy.spin_once(tester, timeout_sec=0.1)
        time.sleep(0.1)
    
    # Stop
    for _ in range(10):
        tester.stop()
        rclpy.spin_once(tester, timeout_sec=0.1)
        time.sleep(0.1)

def main():
    rclpy.init()
    tester = VelocityTester()
    
    try:
        print("Starting comprehensive velocity test...")
        
        # Test different movements
        test_movement(tester, "Moving in +X direction", linear_x=0.1)
        test_movement(tester, "Moving in -X direction", linear_x=-0.1)
        test_movement(tester, "Moving in +Y direction", linear_y=0.1)
        test_movement(tester, "Moving in -Y direction", linear_y=-0.1)
        test_movement(tester, "Moving in +Z direction", linear_z=0.1)
        test_movement(tester, "Moving in -Z direction", linear_z=-0.1)
        test_movement(tester, "Rotating around Z axis", angular_z=0.2)
        test_movement(tester, "Rotating around Z axis (opposite)", angular_z=-0.2)
        
        print("\nTest sequence completed!")
        
    except KeyboardInterrupt:
        print("Test interrupted by user")
        tester.stop()
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()