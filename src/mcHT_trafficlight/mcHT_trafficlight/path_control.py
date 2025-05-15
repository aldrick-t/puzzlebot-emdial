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
import time

class pathControl(Node):
    def __init__(self):
        super().__init__('path_control')
        self.wait_for_ros_time()

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.next_goal_pub = self.create_publisher(Empty, 'next_goal', 10)
        self.pose_sub = self.create_subscription(Pose2D, 'pose', self.pose_cb, 10)
        self.goal_sub = self.create_subscription(Pose2D, 'goal', self.goal_cb, 10)
        self.traffic_light_color_sub = self.create_subscription(String, 'traffic_light_color', self.traffic_light_color_cb, 10)

        # Handle shutdown gracefully
        signal.signal(signal.SIGINT, self.shutdown_function)

        # Logger config
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().debug("Logger set to DEBUG level")
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Declare parameters with descriptors for dynamic reconfigure
        from rcl_interfaces.msg import ParameterDescriptor
        self.robust_margin = self.declare_parameter('robust_margin', 0.9).value
        self.goal_threshold = self.declare_parameter('goal_threshold', 0.02).value
        start_descriptor = ParameterDescriptor(description="Start parameter to trigger the controller")
        self.start = self.declare_parameter('start', False, start_descriptor).value

        # Register dynamic parameter callback once
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.goal_received = False
        self.xg = 0.0 # Goal position x[m]
        self.yg = 0.0 # Goal position y[m]

        self.robot_pose = Pose2D()
        self.xr = 0.0 # Robot position x[m]
        self.yr = 0.0 # Robot position y[m]
        self.theta_r = 0.0 # Robot orientation [rad]

        # Control gains
        self.kp_v = 0.7
        self.ki_v = 0.1
        self.kp_w = 0.7
        self.ki_w = 0.1

        # Limits for integrals (anti-windup)
        self.integral_error_d_max = 1.0
        self.integral_error_theta_max = 1.0

        # Error variables
        self.integral_error_d = 0.0
        self.integral_error_theta = 0.0

        # Traffic color lights
        self.yellow_light = False
        self.red_light = False
        self.green_light = False

        self.moving = True

        # Time
        self.last_time = self.get_clock().now()

        self.cmd_vel = Twist()
        timer_period = 0.02 
        self.create_timer(timer_period, self.main_timer_cb)
        
        self.get_logger().info("Node initialized!!")
        time.sleep(5)
        while not self.start:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.next_goal_pub.publish(Empty()) # Publish empty message to notify next goal
        self.get_logger().info("Requested first Goal")

    def traffic_light_color_cb(self, msg):
        # Check if the ros message is empty
        if msg.data == "":
            self.get_logger().info("No traffic light detected")
            return
        else:
            traffic_light_color = msg.data
            self.get_logger().info(f"Traffic light color detected: {traffic_light_color}")
            if traffic_light_color == "RED":
                self.red_light = True
                self.yellow_light = False
                self.green_light = False
                self.moving = False
            elif traffic_light_color == "YELLOW":
                self.red_light = False
                self.yellow_light = True
                self.green_light = False
            elif traffic_light_color == "GREEN":
                self.red_light = False
                self.yellow_light = False
                self.green_light = True
                self.moving = True

            else:
                self.get_logger().info("Unknown traffic light color detected")
                return

    def main_timer_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now        

        if self.goal_received and self.moving:
            ed, etheta = self.get_errors(self.xr, self.yr, self.xg, self.yg, self.theta_r)

            # Update integrals ONLY if error is not too small
            if abs(ed) > 0.05:
                self.integral_error_d += ed * dt
                self.integral_error_d = np.clip(self.integral_error_d, -self.integral_error_d_max, self.integral_error_d_max)
            else:
                self.integral_error_d = 0.0  # Reset if error is small
                

            if abs(etheta) > 0.05:
                self.integral_error_theta += etheta * dt
                self.integral_error_theta = np.clip(self.integral_error_theta, -self.integral_error_theta_max, self.integral_error_theta_max)
            else:
                self.integral_error_theta = 0.0  # Reset if error is small   
                

            # PI control for linear and angular velocity
            v = self.kp_v * ed + self.ki_v * self.integral_error_d
            w = self.kp_w * etheta + self.ki_w * self.integral_error_theta

            # Saturate speeds
            v = np.clip(v, 0.0, 0.3)
            w = np.clip(w, -1.2, 1.2)

            self.cmd_vel.linear.x = v
            self.cmd_vel.angular.z = w

            if self.yellow_light:
                self.cmd_vel.linear.x *= 0.5
                self.cmd_vel.angular.z *= 1.0
            
            if not self.moving:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0

            # Check if goal is reached
            if ed < self.goal_threshold:
                self.get_logger().info(f"Goal reached: x={self.xg:.2f}, y={self.yg:.2f}")
                
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                
                self.cmd_vel_pub.publish(self.cmd_vel)
                ed = 0.0  # Reset distance error if angle error is small
                etheta = 0.0  # Reset angle error if distance error is small
                
                self.integral_error_d = 0.0
                self.integral_error_theta = 0.0
                self.goal_received = False
                self.next_goal_pub.publish(Empty())
                
                return

            self.cmd_vel_pub.publish(self.cmd_vel)
        
        if self.goal_received and not self.moving:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel)

        else:
            self.get_logger().info("Waiting for goal")
          

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
            elif param.name == 'start' and param.type_ == param.Type.BOOL:
                self.start = param.value
        return SetParametersResult(successful=True)
    
    def update_parameters(self):
        # No static update here; dynamic parameters are handled by parameter_callback.
        pass

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
