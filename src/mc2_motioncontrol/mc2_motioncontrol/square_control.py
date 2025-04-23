'''
Square Path Controller Node with PID Control
The robot moves in a square path, turning at each corner.
The robot stops after completing the square path.
Goal points (each corner) are set via params.
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import numpy as np
import signal # To handle Ctrl+C
import sys # To exit the program

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

        self.goal_received = False
        self.xg = 0.0 # Goal position x[m]
        self.yg = 0.0 # Goal position y[m]

        self.robot_pose = Pose2D()
        self.xr = 0.0 # Robot position x[m]
        self.yr = 0.0 # Robot position y[m]
        self.theta_r = 0.0 # Robot orientation [rad]

        self.kp_v = self.declare_parameter('kp_v', 0.5).get_parameter_value().double_value # Linear velocity gain
        self.kp_w = self.declare_parameter('kp_w', 1.0).get_parameter_value().double_value # Angular velocity gain

        # self.ed = 0.0 # Error in x[m]
        # self.etheta = 0.0 # Error in y[m]
        
        self.cmd_vel = Twist()
        timer_period = 0.05 
        self.create_timer(timer_period, self.main_timer_cb)
        self.get_logger().info("Node initialized!!")


    def pose_cb(self, pose): 
        ## This function receives the /pose from the odometry_node
        self.xr = pose.x
        self.yr = pose.y
        self.theta_r = pose.theta

    def goal_cb(self, goal): 
        ## This function receives the /goal from the path_generator node
        self.xg = goal.x
        self.yg = goal.y
        self.theta_g = goal.theta
        self.goal_received = True
        self.get_logger().info("Goal Received")

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
