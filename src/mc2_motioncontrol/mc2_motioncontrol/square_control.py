'''
Square Path Controller Node with PID Control
The robot moves in a square path, turning at each corner.
The robot stops after completing the square path.
Goal points (each corner) are set via params.
'''

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
import signal 
import sys

class squareControl(Node):
    def __init__(self):
        super().__init__('square_control')
        self.wait_for_ros_time()

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_sub = self.create_subscription(Twist, 'pose', self.pose_callback, 10)
        # Handle shutdown gracefully
        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function

        # Declare parameters
        self.square_side = self.declare_parameter('square_side', 2.0).get_parameter_value().double_value
        self.robust_margin = self.declare_parameter('robust_margin', 0.9).get_parameter_value().double_value


    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('ROS time is active!')

    def shutdown_function(self, signum, frame):
        # Handle shutdown gracefully
        # This function will be called when Ctrl+C is pressed
        # It will stop the robot and shutdown the node
        self.get_logger().info("Shutting down. Stopping robot...")
        stop_twist = Twist()  # All zeros to stop the robot
        self.pub_cmd_vel.publish(stop_twist) # publish it to stop the robot before shutting down
        rclpy.shutdown() # Shutdown the node
        sys.exit(0) # Exit the program
    
def main(args=None):
    rclpy.init(args=args)
    node = squareControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
