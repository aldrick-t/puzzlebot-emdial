'''
robot_ctrl.py

Control Node
Node subsribes to any variable command input and publishes control commands to the robot.
General-use node for multiple applications.
Accepts commands from multiple sources, format must be float from -1.0 to 1.0.
Accepts variable control commands and data inputs.


aldrick-t
MAY 2025
'''
import rclpy
from rclpy.node import Node
import numpy as np
#from cv_bridge import CvBridge
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import SetParametersResult
import time

class RobotCtrl(Node):
    '''
    Control Node
    Subscriptions:
    - /data_input: Data input stream
    - /line_cmd: Line command input
    - /traffic_light_val: Traffic light data input
    Publishers:
    - /cmd_vel: Variable command input
    '''
    
    def __init__(self):
        super().__init__('robot_ctrl')
        #self.bridge = CvBridge()
        
        self.wait_for_ros_time()
        
        # Logger initialization 
        # Set logging level default to DEBUG for all nodes
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG)
        # Start logger
        self.logger = self.get_logger()
        
        #Init message
        self.logger.info("RobotCtrl Starting Initialization...")

        # Parameters immutable after startup sequence
        # None
        
        # Dynamic parameters
        # Logging level
        self.declare_parameter('log_severity', 'DEBUG')
        # Command input topic
        self.declare_parameter('cmd_input_topic', 'line_cmd')
        # Controller gains parameters
        # PID for velocity control
        self.declare_parameter('Kp_v', 0.0)
        self.declare_parameter('Ki_v', 0.0)
        self.declare_parameter('Kd_v', 0.0)
        # PID for angular control
        self.declare_parameter('Kp_w', 0.8) #2.0    #1.4
        self.declare_parameter('Ki_w', 0.0) #1.2    #1.9
        self.declare_parameter('Kd_w', 0.00) #0.5   #0.09
        # Max speed dynamic parameters
        self.declare_parameter('v_limit', 0.3)
        self.declare_parameter('w_limit', 1.0)
        # Max speed slow mode dynamic parameters
        self.declare_parameter('v_limit_slow', 0.2)
        self.declare_parameter('w_limit_slow', 1.0)
        # Absolute maximum safety speed limits
        self.declare_parameter('v_limit_max', 0.7)
        self.declare_parameter('w_limit_max', 1.8)
        # Bend minimum speeds
        self.declare_parameter('v_limit_min', 0.1)
        self.declare_parameter('w_limit_min', 0.1)
        # Activation parameter
        self.declare_parameter('ctrl_activate', False)
        # Curve detect threshhold
        self.declare_parameter('curve_detect_thresh', 0.3)
        
        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Variables
        # Velocity PID gains
        self.Kp_v = self.get_parameter('Kp_v').value
        self.Ki_v = self.get_parameter('Ki_v').value
        self.Kd_v = self.get_parameter('Kd_v').value
        # Angular PID gains
        self.Kp_w = self.get_parameter('Kp_w').value
        self.Ki_w = self.get_parameter('Ki_w').value
        self.Kd_w = self.get_parameter('Kd_w').value
        # Speed limits
        self.v_limit = self.get_parameter('v_limit').value
        self.v_limit_slow = self.get_parameter('v_limit_slow').value
        self.v_limit_max = self.get_parameter('v_limit_max').value
        self.v_limit_min = self.get_parameter('v_limit_min').value
        
        self.w_limit = self.get_parameter('w_limit').value
        self.w_limit_slow = self.get_parameter('w_limit_slow').value
        self.w_limit_max = self.get_parameter('w_limit_max').value
        self.w_limit_min = self.get_parameter('w_limit_min').value

        # Curve detect threshhold
        self.curve_detect_thresh = self.get_parameter('curve_detect_thresh').value
        
        # Init traffic light message var
        self.traffic_light = None
        # Init control activation var
        self.ctrl_activate = self.get_parameter('ctrl_activate').value
        # Traffic light state flags
        self.tl_red = False
        self.tl_yellow = False
        self.tl_green = True
        
        # Subscriptions
        # line command sub
        self.create_subscription(Float32, self.get_parameter('cmd_input_topic').value, self.line_cmd_cb, 10)
        # Traffic light data sub
        self.create_subscription(String, 'traffic_light_color', self.traffic_light_cb, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # cmd_vel twist message init
        self.cmd_vel = Twist()
        # line_cmd message init and flag to ensure a msg is received before moving
        self.line_cmd = 0.0
        self.line_cmd_received = False  # Add flag
        
        # Timer creation
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.last_time = self.get_clock().now()
        
        # Running message
        self.logger.info("RobotCtrl Initialized!")
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'log_severity':
                severity = getattr(LoggingSeverity, param.value)
                self.get_logger().set_level(severity)
                rclpy.logging.get_logger('rclpy').set_level(severity)
                self.get_logger().info(f"Log severity set to {param.value}")
            elif param.name == 'ctrl_activate':
                self.ctrl_activate = param.value
                if self.ctrl_activate:
                    self.get_logger().info("Control system activated")
                else:
                    self.get_logger().info("Control system deactivated, stopping motors.")
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel)
            elif param.name == 'Kp_v':
                self.Kp_v = param.value
                self.get_logger().info(f"Kp_v set to {param.value}")
            elif param.name == 'Kp_w':
                self.Kp_w = param.value
                self.get_logger().info(f"Kp_w set to {param.value}")
            elif param.name == 'Ki_w':
                self.Ki_w = param.value
                self.get_logger().info(f"Ki_w set to {param.value}")
            elif param.name == 'Kd_w':
                self.Kd_w = param.value
                self.get_logger().info(f"Kd_w set to {param.value}")
            elif param.name == 'v_limit':
                self.v_limit = param.value
                self.get_logger().info(f"v_limit set to {param.value}")
            elif param.name == 'w_limit':
                self.w_limit = param.value
                self.get_logger().info(f"w_limit set to {param.value}")
            elif param.name == 'v_limit_slow':
                self.v_limit_slow = param.value
                self.get_logger().info(f"v_limit_slow set to {param.value}")
            elif param.name == 'w_limit_slow':
                self.v_limit_slow = param.value
                self.get_logger().info(f"w_limit_slow set to {param.value}")    
            elif param.name == 'v_limit_min':  
                self.v_limit_min = param.value
                self.get_logger().info(f"v_limit_min set to {param.value}")
            elif param.name == 'w_limit_min':
                self.w_limit_min = param.value
                self.get_logger().info(f"w_limit_min set to {param.value}")
            elif param.name == 'curve_detect_thresh':
                self.curve_detect_thresh = param.value
                self.get_logger().info(f"curve_detect_thresh set to {param.value}")
                
        return SetParametersResult(successful=True)
    
    def line_cmd_cb(self, msg):
        '''
        Callback function for line command input
        '''
        self.line_cmd = msg.data
        self.line_cmd_received = True  # Set flag when message is received
        self.get_logger().debug(f"RECEIVED Line command: {self.line_cmd}", throttle_duration_sec=1.0)
        
    def traffic_light_cb(self, msg):
        '''
        Callback function for traffic light data
        '''
        self.traffic_light = msg.data
        self.get_logger().debug(f"RECEIVED Traffic light data: {self.traffic_light}", throttle_duration_sec=10.0)
        
        #Traffic light state flags
        if self.traffic_light == 'RED':
            self.tl_red = True
            self.tl_yellow = False
            self.tl_green = False
        elif self.traffic_light == 'YELLOW':
            self.tl_red = False
            self.tl_yellow = True
            self.tl_green = False
        elif self.traffic_light == 'GREEN':
            self.tl_red = False
            self.tl_yellow = False
            self.tl_green = True
            self.get_logger().debug("Invalid traffic light data received, defaulting to GREEN.", throttle_duration_sec=5.0)
        

    
    def timer_callback(self):
        '''
        Timer callback function
        Publishes calculated control command to cmd_vel topic
        '''
        if not self.ctrl_activate:
            # self.reset()
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel)
            return

        # Do not move until a line_cmd is received
        if not self.line_cmd_received:
            self.get_logger().info("Waiting for line command message...", throttle_duration_sec=5.0)
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel)
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now 
        
        v = 0.00
        w = 0.00
        
        v, w = self.control_sys(self.line_cmd)
        
        # Conditionals for receieved traffic light data
        if self.tl_red:
            self.get_logger().info("Traffic light RED, stopping robot.", throttle_duration_sec=2.0)
            self.soft_stop()
            return
        elif self.tl_yellow:
            self.get_logger().info("Traffic light YELLOW, slowing down robot.", throttle_duration_sec=2.0)
            # Reduce speed to slow output limits
            v = np.clip(v, 0, (self.v_limit_slow))
            w = np.clip(w, (-self.w_limit_slow), (self.w_limit_slow))
            
        elif self.tl_green:
            self.get_logger().info("Traffic light GREEN, moving robot.", throttle_duration_sec=7.0)
            # Set speed to normal output limits
            v = np.clip(v, 0, (self.v_limit))
            w = np.clip(w, (-self.w_limit), (self.w_limit))
        else:
            pass 
        
        # Publish cmd_vel twist message
        self.cmd_vel.linear.x = v
        self.cmd_vel.angular.z = -w
        self.get_logger().debug(f"Current Linear vel:   {self.cmd_vel.linear.x}", throttle_duration_sec=1.0)
        self.get_logger().debug(f"Current Angular vel:  {self.cmd_vel.angular.z}", throttle_duration_sec=1.0)
        self.cmd_vel_pub.publish(self.cmd_vel)
    
    def control_sys(self, line_cmd):
        '''
        Control system function
        Processes line command and traffic light data to generate control vel command
        '''
        #Timer init
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now 
        
        # Lost Line detection and recovery
        # Check if line is lost, indicated by a command >= abs(1)
        if line_cmd <= -1:
             self.get_logger().debug("Lost line detected, spinning to search for the line")
             # Spin in place: zero linear velocity and fixed turning speed.
             return 0.0, 0.3  #
        elif line_cmd >= 1:
             self.get_logger().debug("Lost line detected, spinning to search for the line")
             # Spin in place: zero linear velocity and fixed turning speed.
             return 0.0, -0.3
        
        # Initialize vars
        # angular error is line_cmd as calculated from LineRecogni and LineCmd
        self.error_w = line_cmd
        # self.get_logger().debug(f"Receieved Line command: {line_cmd}", throttle_duration_sec=1.0)
        
        # Calculate velocity based on angular error
        # If angular error is extreme (near 1 or -1), reduce velocity
        if abs(self.error_w) > self.curve_detect_thresh:
            #vel_x = self.v_limit * 0.6
            vel_x = self.v_limit_min * (1 - abs(self.error_w))  # Reduce velocity proportionally
            self.get_logger().debug(f"Velocity reduced, Ang. Error over Safe Threshold", throttle_duration_sec=5.0)
        else:
            vel_x = self.v_limit  # Maintain maximum velocity in safe range
            self.get_logger().debug(f"Velocity set to max: {vel_x}", throttle_duration_sec=5.0)

        # Calculate angular velocity based on angular error
        # PID control for angular velocity
        self.integral_w = getattr(self, 'integral_w', 0.0) + self.error_w * dt  # Integral term
        derivative_w = (self.error_w - getattr(self, 'prev_error_w', 0.0)) / dt if dt > 0 else 0.0  # Derivative term
        self.prev_error_w = self.error_w  # Update previous error

        vel_w = (
            (self.Kp_w * self.error_w) + 
            (self.Ki_w * self.integral_w) + 
            (self.Kd_w * derivative_w)
        )

        # Clip velocities to their respective limits
        vel_x = np.clip(vel_x, self.v_limit_min, self.v_limit)
        vel_w = np.clip(vel_w, (-self.w_limit), self.w_limit)
        
    
        
        return vel_x, vel_w
    
    
    def reset(self):
        # Reset PID variables
        self.prev_error_w = 0.0
        self.integral_w = 0.0
        self.last_time = None
        
    def soft_stop(self, ramp_time=2.0, rate=0.01):
        '''
        Soft stop function
        Gradually reduces velocity to zero
        '''
        steps = int(ramp_time / rate)
        speed_dec = self.cmd_vel.linear.x / steps
        self.get_logger().info(f"Stopping robot over {ramp_time} seconds.")
        
        for i in range(steps):
            self.cmd_vel.linear.x -= speed_dec
            self.traffic_light_cb
            temp, self.cmd_vel.angular.z = self.control_sys(self.line_cmd)
            self.cmd_vel.angular.z = (-1 * (self.cmd_vel.angular.z))
            self.cmd_vel.angular.z = np.clip(self.cmd_vel.angular.z, (-self.w_limit), self.w_limit)
            self.get_logger().info(f"Current Linear vel:   {self.cmd_vel.linear.x}", throttle_duration_sec=0.0)
            time.sleep(rate)  # Add sleep to control the rate of the loop

        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.get_logger().info(f"Current Linear vel:   {self.cmd_vel.linear.x}", throttle_duration_sec=5.0)
        self.cmd_vel_pub.publish(self.cmd_vel)
        self.get_logger().info("Robot stopped.")
        
    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('ROS time is active!')
        
        
def main(args=None):
    rclpy.init(args=args)
    node = RobotCtrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()