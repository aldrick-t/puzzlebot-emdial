'''
User Defined Path Controller Node with PID Control
The robot moves in the set path, calculating pose and velocity.
The robot stops after completing the path.
Goal points are set via params.
'''

import rclpy
import rclpy.logging
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import numpy as np
import signal # To handle Ctrl+C
import sys # To exit the program
from std_msgs.msg import String


path_left = [0.23, 0.0, 0.25,0.23]
path_right = [0.23, 0.0, 0.25, -0.23]
path_straight = [0.46, 0.0]


class pathControl(Node):
    def __init__(self):
        super().__init__('path_control')
        self.wait_for_ros_time()

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        #self.next_goal_pub = self.create_publisher(Empty, 'next_goal', 10)
        self.pose_sub = self.create_subscription(Pose2D, 'pose', self.pose_cb, 10)
        self.cross_status_sub = self.create_subscription(String, 'cross_status', self.cross_status_cb, 10) # Subscribe to cross pose for debugging
        self.cross_delta_sub = self.create_subscription(String, 'cross_delta_y', self.cross_delta_cb, 10)
        #self.goal_sub = self.create_subscription(Pose2D, 'goal', self.goal_cb, 10)
        
        # Handle shutdown gracefully
        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function

        #logger config
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG) # Set logger to DEBUG level
        self.get_logger().debug("Logger set to DEBUG level")
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG) # Set rclpy logger to DEBUG level

        # Declare parameters
        self.robust_margin = self.declare_parameter('robust_margin', 0.9).get_parameter_value().double_value
        self.goal_threshold = self.declare_parameter('goal_threshold', 0.05).get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.parameter_callback)

        #self.goal_received = False
        self.xg = 0.0 # Goal position x[m]
        self.yg = 0.0 # Goal position y[m]

        self.robot_pose = Pose2D()
        self.xr = 0.0 # Robot position x[m]
        self.yr = 0.0 # Robot position y[m]
        self.theta_r = 0.0 # Robot orientation [rad]

        self.kp_v = self.declare_parameter('kp_v', 0.2).get_parameter_value().double_value # Linear velocity gain
        self.kp_w = self.declare_parameter('kp_w', 1.2).get_parameter_value().double_value # Angular velocity gain
        
        self.cmd_vel = Twist()
        timer_period = 0.05 
        self.create_timer(timer_period, self.main_timer_cb)
        self.get_logger().info("Node initialized!!")

        self.next_goal_pub.publish(Empty()) # Publish empty message to notify next goal
        self.get_logger().info("Requested first Goal")

    def main_timer_cb(self):
        ## This function is called every 0.05 seconds
        if self.goal_received:

            self.get_logger().info("Goal received")
            self.get_logger().info(f"Moving to goal: x={self.xg:.2f}, y={self.yg:.2f}")

            ed, etheta = self.get_errors(self.xr, self.yr, self.xg, self.yg, self.theta_r)

            # Goal Threshold
            if ed < self.goal_threshold: #Threshold value (tolerance) for goal reached in meters.
                self.get_logger().info(f"Goal reached : x={self.xg:.2f}, y={self.yg:.2f}")
                self.get_logger().debug(f"Current pose : x={self.xr:.2f}, y={self.yr:.2f}")
                self.get_logger().debug(f"Current theta: {self.theta_r:.2f}")
                self.get_logger().debug(f"Within thresh: {ed:.2f} m")
                self.goal_received = False
                self.get_logger().debug(f"Goal received: {self.goal_received}")
                self.next_goal_pub.publish(Empty()) # Publish empty message to notify next goal
                self.get_logger().debug("Requested next goal")
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
            else:
                self.cmd_vel.linear.x = self.kp_v * ed
                #limit the linear velocity to a maximum of 0.5 m/s
                if self.cmd_vel.linear.x > 0.5:
                    self.cmd_vel.linear.x = 0.5
                self.get_logger().debug(f"Linear velocity: {self.cmd_vel.linear.x:.2f} m/s")
                if self.cmd_vel.linear.x > 0.5:
                    self.get_logger().warn(f"Linear velocity above safe limit: {self.cmd_vel.linear.x:.2f} m/s")
                self.cmd_vel.angular.z = self.kp_w * etheta
                self.get_logger().debug(f"Angular velocity: {self.cmd_vel.angular.z:.2f} rad/s")
                if self.cmd_vel.angular.z > 1.5:
                    self.get_logger().warn(f"Angular velocity above safe limit: {self.cmd_vel.angular.z:.2f} rad/s")

            self.cmd_vel_pub.publish(self.cmd_vel)
        else: 
            self.get_logger().info("Waiting for goal")
            self.get_logger().debug(f"Goal received: {self.goal_received}")

            

    def get_errors(self, xr, yr, xg, yg, theta_r):
        ## This function computes the errors in x and y
        # Compute the distance to the goal
        ed = np.sqrt((xg - xr)**2 + (yg - yr)**2)
        # Compute the angle to the goal
        thetag = np.arctan2(yg - yr, xg - xr)
        etheta = thetag - theta_r
        # Normalize the angle to be between -pi and pi
        etheta = np.arctan2(np.sin(etheta), np.cos(etheta))
        # Debug prints
        self.get_logger().debug(f"Distance to goal: {ed:.2f} m")
        self.get_logger().debug(f"Angle to goal: {etheta:.2f} rad")
        return ed, etheta

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

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'robust_margin' and param.type_ == param.Type.DOUBLE:
                self.robust_margin = param.value
                self.get_logger().info(f"Updated robust_margin: {self.robust_margin}")
            elif param.name == 'goal_threshold' and param.type_ == param.Type.DOUBLE:
                self.goal_threshold = param.value
                self.get_logger().info(f"Updated goal_threshold: {self.goal_threshold}")
            elif param.name == 'kp_v' and param.type_ == param.Type.DOUBLE:
                self.kp_v = param.value
                self.get_logger().info(f"Updated kp_v: {self.kp_v}")
            elif param.name == 'kp_w' and param.type_ == param.Type.DOUBLE:
                self.kp_w = param.value
                self.get_logger().info(f"Updated kp_w: {self.kp_w}")
        return SetParametersResult(successful=True)

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
    node = pathControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
