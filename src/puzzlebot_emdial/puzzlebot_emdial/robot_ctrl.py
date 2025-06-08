'''
robot_ctrl.py

Control Node
Node subsribes to any variable command input and publishes control commands to the robot.
General-use node for multiple applications.
Accepts commands from multiple sources, format must be float from -1.0 to 1.0.
Accepts variable control commands and data inputs.

emdial
MAY 2025
'''
import rclpy
from rclpy.node import Node
import numpy as np
#from cv_bridge import CvBridge
from std_msgs.msg import Float32, String, Int32
from geometry_msgs.msg import Twist, Pose2D
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
        self.declare_parameter('Kp_w', 0.58) #2.0    #1.4
        self.declare_parameter('Ki_w', 0.0) #1.2    #1.9
        self.declare_parameter('Kd_w', 0.00) #0.5   #0.09
        # Max speed dynamic parameters
        self.declare_parameter('v_limit', 0.15)
        self.declare_parameter('w_limit', 1.0)
        # Max speed slow mode dynamic parameters
        self.declare_parameter('v_limit_slow', 0.05)
        self.declare_parameter('w_limit_slow', 1.0)
        # Absolute maximum safety speed limits
        self.declare_parameter('v_limit_max', 0.7)
        self.declare_parameter('w_limit_max', 1.8)
        # Bend minimum speeds
        self.declare_parameter('v_limit_min', 0.05)
        self.declare_parameter('w_limit_min', 0.1)
        # Activation parameter
        self.declare_parameter('ctrl_activate', False)
        # Traffic detection toggle
        self.declare_parameter('detect_tl', True)
        # Traffic sign detection toggle
        self.declare_parameter('detect_ts', True)
        # Cancel path trigger
        self.declare_parameter('cancel_path', False)
        # Curve detect threshhold
        self.declare_parameter('curve_detect_thresh', 0.3)
        # Parameter Callback
        
        # Temp Params for Preset Path tuning
        # Left Turn Path
        self.declare_parameter('pathL_1', 0.20)
        self.declare_parameter('pathL_2', 0.0)
        self.declare_parameter('pathL_3', 0.31)
        self.declare_parameter('pathL_4', 0.20)
        # Right Turn Path
        self.declare_parameter('pathR_1', 0.23)
        self.declare_parameter('pathR_2', 0.0)
        self.declare_parameter('pathR_3', 0.33)
        self.declare_parameter('pathR_4', -0.20)
        # Straight Path
        self.declare_parameter('pathS_1', 0.40)
        self.declare_parameter('pathS_2', 0.0)
        
        
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
        # Control activation var
        self.ctrl_activate = self.get_parameter('ctrl_activate').value
        # Traffic detection toggle
        self.detect_tl = self.get_parameter('detect_tl').value
        # Traffic sign detection toggle
        self.detect_ts = self.get_parameter('detect_ts').value
        # Cancel Path trig
        self.cancel_path_toggle = self.get_parameter('cancel_path').value
        
        # Init traffic light message var
        self.traffic_light = None
        # Traffic light state flags
        self.tl_red = False
        self.tl_yellow = False
        self.tl_green = True
        self.moving = True

        #Paths and TS
        self.ts_left = False
        self.ts_right = False
        self.ts_straight = False
        
        self.ts_stop = False
        self.ts_give = False
        self.ts_work = False

        self.ts_start_time_reduce_speed = None
        self.reduced_sign_speed = False

        self.ts_stop_time = None
        self.stoppin_flag = False

        # Path
        self.goal_received = False
        self.current_goal_index = -1

        # Preset Paths 
        # self.path_left = [0.23, 0.0, 0.31, 0.24]
        # self.path_right = [0.23, 0.0, 0.33, -0.24]
        # self.path_straight = [0.40, 0.0]
        
        # Load paths from parameters (for tuning)
        self.path_left = [
            self.get_parameter('pathL_1').value,
            self.get_parameter('pathL_2').value,
            self.get_parameter('pathL_3').value,
            self.get_parameter('pathL_4').value
        ]
        self.path_right = [
            self.get_parameter('pathR_1').value,
            self.get_parameter('pathR_2').value,
            self.get_parameter('pathR_3').value,
            self.get_parameter('pathR_4').value
        ]
        self.path_straight = [
            self.get_parameter('pathS_1').value,
            self.get_parameter('pathS_2').value
        ]
        

        self.path = self.path_left

        # Cross 
        self.approach = False
        self.xing = False
        self.no_cross = False
        
        # "Local"
        self.crossing = False

        self.centroid_delta = 120

        self.xg = 0.0 # Goal position x[m]
        self.yg = 0.0 # Goal position y[m]
        #--------------------------------------------------

        # Subscriptions
        # line command sub
        self.create_subscription(Float32, self.get_parameter('cmd_input_topic').value, self.line_cmd_cb, 10)
        # Traffic light data sub
        self.create_subscription(String, 'traffic_light', self.traffic_light_cb, 10)
        # Traffic sign data sub
        self.create_subscription(String, 'traffic_sign', self.traffic_sign_cb, 10)
        # Cross status
        self.cross_status_sub = self.create_subscription(String, 'cross_status', self.cross_status_cb, 10)
        # Zebra centroids and y-delta
        self.cross_delta_sub = self.create_subscription(Int32, 'delta_y', self.delta_y_cb, 10)
        # Odometry calculated Pose
        self.pose_sub = self.create_subscription(Pose2D, 'pose', self.pose_cb, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.reset_odom_pub = self.create_publisher(String, 'reset_odometry', 10)
        
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
            elif param.name == 'detect_tl':
                self.detect_tl = param.value
                if self.detect_tl:
                    self.get_logger().info("Traffic light detection ENABLED")
                else:
                    self.get_logger().info("Traffic light detection DISABLED")
            elif param.name == 'detect_ts':
                self.detect_ts = param.value
                if self.detect_ts:
                    self.get_logger().info("Traffic sign detection ENABLED")
                else:
                    self.get_logger().info("Traffic sign detection DISABLED")
            elif param.name == 'cancel_path':
                self.cancel_path_toggle = param.value
                if self.cancel_path_toggle:
                    self.get_logger().info("Cancel Path TRIGGERED")
                else:
                    self.get_logger().info("Cancel Path UNTRIGGERED")
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
                self.w_limit_slow = param.value
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
            # Live path tuning parameters
            elif param.name == 'pathL_1':
                self.path_left[0] = param.value
                self.get_logger().info(f"pathL_1 set to {param.value}")
            elif param.name == 'pathL_2':
                self.path_left[1] = param.value
                self.get_logger().info(f"pathL_2 set to {param.value}")
            elif param.name == 'pathL_3':
                self.path_left[2] = param.value
                self.get_logger().info(f"pathL_3 set to {param.value}")
            elif param.name == 'pathL_4':
                self.path_left[3] = param.value
                self.get_logger().info(f"pathL_4 set to {param.value}")
            elif param.name == 'pathR_1':
                self.path_right[0] = param.value
                self.get_logger().info(f"pathR_1 set to {param.value}")
            elif param.name == 'pathR_2':
                self.path_right[1] = param.value
                self.get_logger().info(f"pathR_2 set to {param.value}")
            elif param.name == 'pathR_3':
                self.path_right[2] = param.value
                self.get_logger().info(f"pathR_3 set to {param.value}")
            elif param.name == 'pathR_4':
                self.path_right[3] = param.value
                self.get_logger().info(f"pathR_4 set to {param.value}")
            elif param.name == 'pathS_1':
                self.path_straight[0] = param.value
                self.get_logger().info(f"pathS_1 set to {param.value}")
            elif param.name == 'pathS_2':
                self.path_straight[1] = param.value
                self.get_logger().info(f"pathS_2 set to {param.value}")
            # Live path tuning parameters end    
        
        return SetParametersResult(successful=True)
    

    def line_cmd_cb(self, msg):
        '''
        Callback function for line command input
        '''
        self.line_cmd = msg.data
        
        if self.line_cmd == 1500:
            self.get_logger().warn("Received None line command, setting to 0.0", throttle_duration_sec=10.0)
            self.line_cmd = 0.0
        
        self.line_cmd_received = True  # Set flag when message is received
        self.get_logger().debug(f"RECEIVED Line command: {self.line_cmd}", throttle_duration_sec=10.0)
    

    def cross_status_cb(self, msg):
        '''
        Callback function for cross status
        '''
        if (msg.data =='xing'):
            self.xing = True
        elif (msg.data == 'approach'):
            self.approach = True
        else:
            pass
        
        self.get_logger().debug(f"RECEIVED CROSS command: {msg.data}", throttle_duration_sec=1.0)
    

    def delta_y_cb(self, msg):
        '''
        Callback function for delta y
        '''
        if self.xing == True: 
            self.centroid_delta = msg.data
            self.get_logger().debug(f"RECEIVED Zebra Delta: {self.centroid_delta}", throttle_duration_sec=1.0)
        else:
            pass
    

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
        self.get_logger().debug(f"Distance to goal: {ed:.2f} m", throttle_duration_sec=1.0)
        self.get_logger().debug(f"Angle to goal: {etheta:.2f} rad", throttle_duration_sec=1.0)
        return ed, etheta


    def pose_cb(self, pose): 
        ## This function receives the /pose from the odometry_node
        self.xr = pose.x
        self.yr = pose.y
        self.theta_r = pose.theta


    def start_path(self, path):
        self.path = path
        self.current_goal_index = 0
        

    def give_point(self):
        if self.current_goal_index + 1 >= len(self.path):
            self.get_logger().info('Reached end of path, no more points.')
            self.xing = False
            self.approach = False
            self.crossing = False
            self.current_goal_index = -1
        else:
            self.xg = self.path[self.current_goal_index] # Goal position x[m]
            self.yg = self.path[self.current_goal_index + 1] # Goal position y[m]
            self.goal_received = True

    def cancel_path(self):
        self.get_logger().info("Path CANCELLED, stopping robot.")
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel)
        self.goal_received = False
        self.current_goal_index = -1
        self.xing = False
        self.approach = False
        self.crossing = False
        

    def odometry(self):
        # Start the selected path only if no goal is active yet
        if not self.goal_received and self.current_goal_index < 0:
            if self.ts_left:
                self.get_logger().info("Left path selected.")
                self.start_path(self.path_left)

            elif self.ts_right:
                self.get_logger().info("Right path selected.")
                self.start_path(self.path_right)

            elif self.ts_straight:
                self.get_logger().info("Straight path selected.")
                self.start_path(self.path_straight)
            
            cross_msg = String()
            cross_msg.data = "reset"
            self.reset_odom_pub.publish(cross_msg)

        if not self.goal_received:
            self.give_point()

        # Now continue path following if a goal was received
        if self.goal_received:
            ed, etheta = self.get_errors(self.xr, self.yr, self.xg, self.yg, self.theta_r)

            # Goal Threshold
            if ed < 0.05: #Threshold value (tolerance) for goal reached in meters.
                self.get_logger().info(f"Goal reached : x={self.xg:.2f}, y={self.yg:.2f}")
                self.get_logger().debug(f"Current pose : x={self.xr:.2f}, y={self.yr:.2f}")
                self.get_logger().debug(f"Current theta: {self.theta_r:.2f}")
                self.get_logger().debug(f"Within thresh: {ed:.2f} m")
                self.get_logger().debug(f"Goal received: {self.goal_received}")
                self.get_logger().debug("Requested next goal")
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.goal_received = False
                self.current_goal_index +=2
            else:
                self.cmd_vel.linear.x = 0.4 * ed
                #limit the linear velocity to a maximum of 0.2 m/s
                if self.cmd_vel.linear.x > 0.2:
                    self.cmd_vel.linear.x = 0.2
                self.get_logger().debug(f"Linear velocity: {self.cmd_vel.linear.x:.2f} m/s", throttle_duration_sec=1.0)

                self.cmd_vel.angular.z = 1.2 * etheta
                self.get_logger().debug(f"Angular velocity: {self.cmd_vel.angular.z:.2f} rad/s", throttle_duration_sec=1.0)
                if self.cmd_vel.angular.z > 1.2:
                    self.cmd_vel.angular.z = 1.2
                    self.get_logger().warn(f"Angular velocity above safe limit: {self.cmd_vel.angular.z:.2f} rad/s", throttle_duration_sec=1.0)

            self.cmd_vel_pub.publish(self.cmd_vel)
        else: 
            self.get_logger().info("Waiting for goal")
            self.get_logger().debug(f"Goal received: {self.goal_received}")
        
        
    def traffic_light_cb(self, msg):
        """
        Callback function for traffic light data.
        Accepts msg.data as either:
        - a single color string, e.g. "red"
        - a comma-separated list, e.g. "red, green, yellow"
        """
        raw = msg.data or ""
        # parse into a list of color tokens
        colors = [c.strip().lower() for c in raw.split(',') if c.strip()]
        self.get_logger().debug(f"RECEIVED Traffic light data: {colors}", throttle_duration_sec=5.0)        

        # set flags based on which colors are present
        if 'tl_red' in colors:
            self.tl_red = True
            self.tl_yellow = False
            self.tl_green = False
            self.moving = False
        elif 'tl_yellow' in colors:
            self.tl_yellow = True
            self.tl_red = False
            self.tl_green = False
        elif 'tl_green' in colors:
            self.tl_green = True
            self.moving = True
            self.tl_red = False
            self.tl_yellow = False

        if 'none' in colors:
            self.get_logger().debug(f"None detected, last color state remains.", throttle_duration_sec=5.0)
            
        self.traffic_light = colors
    

    def traffic_sign_cb(self, msg):
        """
        Callback function for traffic sign data.
        Accepts msg.data as either:
        - a single color string, e.g. "red"
        - a comma-separated list, e.g. "red, green, yellow"
        """
        raw = msg.data or ""
        # parse into a list of color tokens
        signs = [c.strip().lower() for c in raw.split(',') if c.strip()]
        self.get_logger().debug(f"RECEIVED Traffic Sign data: {signs}", throttle_duration_sec=1.0)        

        # set flags based on which signs are present
        if 'ts_stop' in signs:
            self.ts_left = False
            self.ts_right = False
            self.ts_straight = False
            self.ts_stop = True
            self.ts_give = False
            self.ts_work = False
            self.moving = False
        elif 'ts_giveway' in signs:
            self.ts_left = False
            self.ts_right = False
            self.ts_straight = False
            self.ts_stop = False
            self.ts_give = True
            self.ts_work = False
        elif 'ts_work' in signs:
            self.ts_left = False
            self.ts_right = False
            self.ts_straight = False
            self.ts_stop = False
            self.ts_give = False
            self.ts_work = True
        elif 'ts_left' in signs:
            self.ts_left = True
            self.ts_right = False
            self.ts_straight = False
            self.ts_stop = False
            self.ts_give = False
            self.ts_work = False
        elif 'ts_right' in signs:
            self.ts_left = False
            self.ts_right = True
            self.ts_straight = False
            self.ts_stop = False
            self.ts_give = False
            self.ts_work = False
        elif 'ts_straight' in signs:
            self.ts_left = False
            self.ts_right = False
            self.ts_straight = True
            self.ts_stop = False
            self.ts_give = False
            self.ts_work = False

        if 'none' in signs:
            self.get_logger().debug(f"None detected, last color state remains.", throttle_duration_sec=1.0)
            
        self.traffic_light = signs
        
    def timer_callback(self):
        '''
        Timer callback function
        Publishes calculated control command to cmd_vel topic
        '''
        if not self.ctrl_activate:
            # Control deactivated: stop immediately
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
        
        # If line command is zero cancel path and stop
        if self.cancel_path_toggle:
            self.cancel_path()

            return
        
        if self.xing and self.approach and not self.crossing:
            if self.tl_green:
                #Stop motors before odometry
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)
                self.approach = False
                self.xing = False
                self.crossing = True
            if self.tl_red:
                self.get_logger().info("Traffic light RED, stopping robot.", throttle_duration_sec=1.0)
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)
                return

                
        if self.crossing:
            self.odometry()
            return
        
        
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if self.detect_ts and self.ts_stop:
            if self.ts_stop_time is None:
                self.ts_stop_time = now
                self.get_logger().info("STOOOP: Stopping for 10 seconds")

            elapsed_secs = (now - self.ts_stop_time).nanoseconds * 1e-9
            if elapsed_secs <= 10.0:
                self.stoppin_flag = True
            else:
                self.ts_work = False
                self.stoppin_flag = False
                self.ts_stop_time = None
                self.get_logger().info("STOOP: 10 Seconds ELAPSED")

        if self.detect_ts and self.ts_work:
            if self.ts_start_time_reduce_speed is None:
                self.ts_start_time_reduce_speed = now
                self.get_logger().info("Working: Reduce Speed")

            elapsed_secs = (now - self.ts_start_time_reduce_speed).nanoseconds * 1e-9
            if elapsed_secs <= 10.0:
                self.reduced_sign_speed = True
            else:
                self.ts_work = False
                self.reduced_sign_speed = False
                self.ts_start_time_reduce_speed = None
                self.get_logger().info("Setting Normal Speed")

        # If we see a 'give way' sign and detect_ts == True, reduce speed for 10 seconds:
        elif self.detect_ts and self.ts_give:
            if self.ts_start_time_reduce_speed is None:
                self.ts_start_time_reduce_speed = now
                self.get_logger().info("Give Way: Reduce Speed")

            elapsed_secs = (now - self.ts_start_time_reduce_speed).nanoseconds * 1e-9
            if elapsed_secs <= 5.0:
                self.reduced_sign_speed = True
            else:
                self.ts_give = False
                self.reduced_sign_speed = False
                self.ts_start_time_reduce_speed = None
                self.get_logger().info("Setting Normal Speed")

        else:
            # If detect_ts == False, force all sign-based reductions off:
            if not self.detect_ts:
                self.reduced_sign_speed = False
                self.ts_start_time_reduce_speed = None
        
        # Compute base control from line following
        v, w = self.control_sys(self.line_cmd)
        
        # If traffic detection disabled, publish line-following commands directly
        if not self.detect_tl:
            # Clip to normal limits
            v = np.clip(v, 0, self.v_limit)
            w = np.clip(w, -self.w_limit, self.w_limit)
            self.cmd_vel.linear.x = v
            self.cmd_vel.angular.z = -w
            self.get_logger().debug(f"[Line only] Linear vel: {self.cmd_vel.linear.x}", throttle_duration_sec=1.0)
            self.get_logger().debug(f"[Line only] Angular vel: {self.cmd_vel.angular.z}", throttle_duration_sec=1.0)
            self.cmd_vel_pub.publish(self.cmd_vel)
            return

        # Traffic detection enabled: apply traffic light logic
        if self.stoppin_flag:
            self.get_logger().info("Traffic Sign STOP, stopping robot.", throttle_duration_sec=1.0)
            #self.soft_stop()
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel)
            return
        elif self.tl_yellow or self.reduced_sign_speed:
            self.get_logger().info("Traffic light YELLOW, slowing down robot.", throttle_duration_sec=1.0)
            if self.moving:
                v = np.clip(v, 0, self.v_limit_slow)
            w = np.clip(w, -self.w_limit_slow, self.w_limit_slow)
        elif self.tl_green:
            self.get_logger().info("Traffic light GREEN, moving robot.", throttle_duration_sec=1.0)
            v = np.clip(v, 0, self.v_limit)
            w = np.clip(w, -self.w_limit, self.w_limit)
        # else: no change, proceed with base v,w
        
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
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now 
        
        # Lost Line detection and recovery
        # Disabled when Intersection is in Approach or Crossing state
        # Check if line is lost, indicated by a command >= abs(1)
        # if not self.approach or not self.crossing or not self.xing:
        #     if line_cmd <= -1 :
        #         self.get_logger().debug("Lost line detected, spinning to search for the line")
        #         # Spin in place: zero linear velocity and fixed turning speed.
        #         return 0.0, 0.3  #
        #     elif line_cmd >= 1:
        #         self.get_logger().debug("Lost line detected, spinning to search for the line")
        #         # Spin in place: zero linear velocity and fixed turning speed.
        #         return 0.0, -0.3
        
        # Initialize variables
        self.error_w = line_cmd
        
        # Calculate velocity based on angular error
        if abs(self.error_w) > self.curve_detect_thresh:
            vel_x = self.v_limit_min * (1 - abs(self.error_w))  # Reduce velocity proportionally
            self.get_logger().debug(f"Velocity reduced, Ang. Error over Safe Threshold", throttle_duration_sec=5.0)
        else:
            vel_x = self.v_limit  # Maintain maximum velocity in safe range
            self.get_logger().debug(f"Velocity set to max: {vel_x}", throttle_duration_sec=5.0)

        # Calculate angular velocity based on angular error
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
        vel_w = np.clip(vel_w, -self.w_limit, self.w_limit)
        
        return vel_x, vel_w
    
    def reset(self):
        # Reset PID variables
        self.prev_error_w = 0.0
        self.integral_w = 0.0
        self.last_time = None
        
    def soft_stop(self, ramp_time=0.5, rate=0.005):
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
            self.cmd_vel.angular.z = -self.cmd_vel.angular.z
            self.cmd_vel.angular.z = np.clip(self.cmd_vel.angular.z, -self.w_limit, self.w_limit)
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
