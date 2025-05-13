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
        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function

        #logger config
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG) # Set logger to DEBUG level
        self.get_logger().debug("Logger set to DEBUG level")
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG) # Set rclpy logger to DEBUG level

        # Declare parameters
        self.robust_margin = self.declare_parameter('robust_margin', 0.9).get_parameter_value().double_value
        self.goal_threshold = self.declare_parameter('goal_threshold', 0.03).get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.goal_received = False
        self.xg = 0.0 # Goal position x[m]
        self.yg = 0.0 # Goal position y[m]

        self.robot_pose = Pose2D()
        self.xr = 0.0 # Robot position x[m]
        self.yr = 0.0 # Robot position y[m]
        self.theta_r = 0.0 # Robot orientation [rad]

        # self.kp_v = self.declare_parameter('kp_v', 0.2).get_parameter_value().double_value # Linear velocity gain
        # self.kp_w = self.declare_parameter('kp_w', 1.2).get_parameter_value().double_value # Angular velocity gain
        
        # Control gains
        self.kp_v = 0.4
        self.ki_v = 0.2
        self.kp_w = 2.0
        self.ki_w = 0.05
        self.kd_w = 0.2

        # Limits for integrals (anti-windup)
        self.integral_error_d_max = 1.0
        self.integral_error_theta_max = 1.0
        self.prev_error_theta = 0.0

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

            # Discrete PI control for Linear Velocity (ed)
            # --- Update errors history ---
            if not hasattr(self, 'e_d'):
                self.e_d = [0.0, 0.0, 0.0]  # current, prev, prev2
                self.u_d = [0.0, 0.0]        # current, prev

            self.e_d = [ed, self.e_d[0], self.e_d[1]]

            # Coefficients for Discrete PI (K1, K2, K3 for linear velocity)
            T = dt  # Use real sampling time
            kp_v = self.kp_v
            ki_v = 0.0
            kd_v = self.ki_v  # Derivative not used in linear velocity
            K1_v = kp_v + T * ki_v + kd_v / T
            K2_v = -kp_v - 2.0 * kd_v / T
            K3_v = kd_v / T

            # Compute control output
            u_d_current = self.u_d[0] + K1_v * self.e_d[0] + K2_v * self.e_d[1] + K3_v * self.e_d[2]

            # Saturate velocity
            v = np.clip(u_d_current, 0.0, 0.4)
            #v *= 0.95
            self.u_d = [u_d_current, self.u_d[0]]

            # Discrete PID control for Angular Velocity (etheta)
            if not hasattr(self, 'e_theta'):
                self.e_theta = [0.0, 0.0, 0.0]  # current, prev, prev2
                self.u_theta = [0.0, 0.0]        # current, prev

            self.e_theta = [etheta, self.e_theta[0], self.e_theta[1]]

            # Coefficients for Discrete PID (K1, K2, K3 for angular velocity)
            kp_w = self.kp_w
            ki_w = 0.0
            kd_w = self.kd_w
            K1_w = kp_w + T * ki_w + kd_w / T
            K2_w = -kp_w - 2.0 * kd_w / T
            K3_w = kd_w / T

            # Compute control output
            u_theta_current = self.u_theta[0] + K1_w * self.e_theta[0] + K2_w * self.e_theta[1] + K3_w * self.e_theta[2]

            # Saturate angular velocity
            w = np.clip(u_theta_current, -0.9, 0.9)
            self.u_theta = [u_theta_current, self.u_theta[0]]

            # Traffic light effect
            if self.yellow_light:
                v *= 0.5

            # Publish cmd_vel
            self.cmd_vel.linear.x = v
            self.cmd_vel.angular.z = w
            self.cmd_vel_pub.publish(self.cmd_vel)

            # Check if goal is reached
            if ed < self.goal_threshold:
                self.get_logger().info(f"Goal reached: x={self.xg:.2f}, y={self.yg:.2f}")
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)
                self.goal_received = False

                # Reset control states
                self.e_d = [0.0, 0.0, 0.0]
                self.u_d = [0.0, 0.0]
                self.e_theta = [0.0, 0.0, 0.0]
                self.u_theta = [0.0, 0.0]

                self.next_goal_pub.publish(Empty())
        elif self.goal_received and not self.moving:
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
