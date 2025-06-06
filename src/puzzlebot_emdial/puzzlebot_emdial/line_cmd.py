'''
line_cmd.py
Refactored for puzzlebot_emdial package.

Line Command Node
Node subscribes to LineRecogni topics and publishes commands based on line detection.

Base by: aldrick-t
Enhanced by: Team emdial
JUN 2025
'''

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, Int32MultiArray, String
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import SetParametersResult

class LineCmd(Node):
    '''
    Line Command Node
    Subscriptions:
    - /line_overlay: Overlay image with detected lines
    - /line_recogni: Proximal Line recognition data
    - /line_recogni_mid: Midrange line recognition data
    
    Publishers:
    - /line_cmd: Line following command
    '''
    def __init__(self):
        super().__init__('line_cmd')
        self.bridge = CvBridge()
        
        self.wait_for_ros_time()
        
        # Logger initialization 
        # Set logging level default to DEBUG for all nodes
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG)
        # Start logger
        self.logger = self.get_logger()
        
        #Init message
        self.logger.info("LineCmd Starting Initialization...")
        
        # Parameters immutable after startup sequence
        # None
        
        # Dynamic parameters
        # Logging level
        self.declare_parameter('log_severity', 'DEBUG')
        self.declare_parameter('dy_precheck_thresh', 0.00)
        
        # Parameter vars
        self.dy_precheck_thresh = self.get_parameter('dy_precheck_thresh').get_parameter_value().double_value
        
        # Flags
        self.cross_in_fov = False
        self.cross_in_prox = False
        
        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Subscriptions
        # Image stream topic
        # self.create_subscription(Image, 'lr_overlay', self.img_cb, 10)
        # Data stream topic
        self.create_subscription(Int32, 'line_recogni', self.line_cmd_cb, 10)
        self.create_subscription(Int32MultiArray, 'line_recogni_mid_cx', self.lr_mid_cx_cb, 10)
        self.create_subscription(Int32MultiArray, 'line_recogni_mid_cy', self.lr_mid_cy_cb, 10)
        # Image dimension topic
        self.create_subscription(Int32MultiArray, 'viewfield_dim', self.viewfield_dim_cb, 10)
        
        # Publishers
        # Line following interpreted command
        self.line_cmd_pub = self.create_publisher(Float32, 'line_cmd', 10)
        # cross detection command
        self.cross_detect_pub = self.create_publisher(String, 'cross_status', 10)
        # Delta Y value for crossing detection
        self.delta_y_pub = self.create_publisher(Float32, 'delta_y', 10)
        
        # Data arrays
        self.viewfield_dim_array = []
        
        # Timer for periodic processing
        self.create_timer(0.01, self.timer_process_cb)
        # Timer callback function
        
        # Running message
        self.logger.info("LineRecogni Initialized!")
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'log_severity':
                severity = getattr(LoggingSeverity, param.value)
                self.get_logger().set_level(severity)
                rclpy.logging.get_logger('rclpy').set_level(severity)
                self.get_logger().info(f"Log severity set to {param.value}")
            elif param.name == 'dy_precheck_thresh':
                self.dy_precheck_thresh = param.value
                self.get_logger().info(f"Dynamic precheck threshold set to {self.dy_precheck_thresh}")
            else:
                self.get_logger().warn(f"Unknown parameter: {param.name}")
        
        return SetParametersResult(successful=True)
    
    def lr_mid_cx_cb(self, msg):
        '''
        Callback function for midrange line recognition topic
        '''
        # Parse message data
        self.cx_array = np.array(msg.data)
    
    def lr_mid_cy_cb(self, msg):
        '''
        Callback function for midrange line recognition topic
        '''
        # Parse message data
        self.cy_array = np.array(msg.data)
        
    def viewfield_dim_cb(self, msg):
        '''
        Callback function for all frame dimensions topic
        '''
        self.viewfield_dim_array = np.array(msg.data)
        
    def line_cmd_cb(self, msg):
        '''
        Callback function for line command topic
        '''
        # Gather data from source image processing
        if (isinstance(self.viewfield_dim_array, np.ndarray) and self.viewfield_dim_array.size == 0) or (isinstance(self.viewfield_dim_array, list) and len(self.viewfield_dim_array) == 0):
            self.get_logger().warn("Viewfield dimensions not available, cannot process line command.")
            return
            
        full_width, center_x, null_thresh_l, null_thresh_r = self.process_sources(self.viewfield_dim_array)
        process_img_data = (full_width, center_x, null_thresh_l, null_thresh_r)
        # Process line recognition data
        line_recogni_data = msg.data
        # Process line command
        cmd_msg = self.process_line_cmd(process_img_data, line_recogni_data)
        # Publish command message
        cmd_msg = cmd_msg.data
        if cmd_msg > 1.1:
            cmd_msg -= int(cmd_msg)
            cmd_msg = 1.0 - cmd_msg
            cmd_msg *= -1.0
        elif cmd_msg < -1.1:
            cmd_msg += int(cmd_msg) * -1.0
            cmd_msg = 1.0 + cmd_msg
        cmd_msg_t = Float32()
        cmd_msg_t.data = cmd_msg
        self.line_cmd_pub.publish(cmd_msg_t)
        
    def timer_process_cb(self):
        '''
        Timer callback function for periodic processing
        '''
        # Check if viewfield dimensions are available
        if (isinstance(self.viewfield_dim_array, np.ndarray) and self.viewfield_dim_array.size == 0) or (isinstance(self.viewfield_dim_array, list) and len(self.viewfield_dim_array) == 0):
            self.get_logger().warn("Viewfield dimensions not available, cannot process line command.")
            return

        full_width, center_x, null_thresh_l, null_thresh_r = self.process_sources(self.viewfield_dim_array)
        
        # Check if cx_array and cy_array are available
        process_img_data = (full_width, center_x, null_thresh_l, null_thresh_r)
        if hasattr(self, 'cx_array') and hasattr(self, 'cy_array'):
            # Process crossing detection
            self.process_cross(self.cx_array, self.cy_array, process_img_data[1])
            
        else:
            self.get_logger().warn("Centroid arrays not available for crossing detection.")
        
    def process_sources(self, viewfield_dim_array):
        '''
        Process the source image to extract information
        '''
        # Get image dimensions
        # Extract full frame dimensions
        full_height, full_width = viewfield_dim_array[0], viewfield_dim_array[1]
        # Extract proximal frame dimensions
        proximal_height, proximal_width = viewfield_dim_array[2], viewfield_dim_array[3]
        # Extract midrange frame dimensions
        midrange_height, midrange_width = viewfield_dim_array[4], viewfield_dim_array[5]
        
        # calculate center of the image
        center_x = full_width // 2
        
        # Calculate null thresholds
        # Define null thresholds as a percentage of the image width
        # null_thresh_l: % from the left edge of the image
        # null_thresh_r: % from the left edge of the image (or % from the right edge)
        null_thresh_l = int(center_x * 0.98)
        null_thresh_r = int(center_x * 1.02)
        
        # Add to overlay image
        return full_width, center_x, null_thresh_l, null_thresh_r
        
        
    def process_line_cmd(self, process_img_data, line_recogni_data):
        '''
        Process the line command based on collected line recognition data
        Outputs cmd value in a range from -1 to 1 based on line position relative to image center
        Considers set thresholds for null zones (set cmd to 0)
        '''
        # Initialize command message
        cmd_msg = Float32()
        # Extract data from process image
        width, center_x, null_thresh_l, null_thresh_r = process_img_data
        # Extract line recognition data
        line_recogni = line_recogni_data
        # Calculate command value based on line position
        if line_recogni > null_thresh_l and line_recogni < null_thresh_r:
            # Line is within the null zone
            cmd_msg.data = 0.0
        elif line_recogni < null_thresh_l:
            # Line is to the left of the null zone
            cmd_msg.data = -1.0 * ((center_x - line_recogni) / center_x) 
        elif line_recogni > null_thresh_r:
            # Line is to the right of the null zone
            cmd_msg.data = 1.0 * ((line_recogni - center_x) / center_x)
            
        # Log command value
        self.get_logger().debug(f"Line command: {cmd_msg.data}", throttle_duration_sec=20.0)
        
        return cmd_msg
        
    def process_cross(self, cx_array, cy_array, center_x):
        '''
        Detection verification for crossing lines
        '''
        cross_msg = String()
        
        # Check if cx_array and cy_array have enough data
        if len(cx_array) < 4 or len(cy_array) < 4:
            #self.get_logger().warn("Not enough centroids detected for crossing verification.")
            cross_msg.data = "none"
            self.cross_detect_pub.publish(cross_msg)
            return
        '''
        Process centroids to filter noise
        Filter centroids based on a flexible threshold 
        Threshold calculated from distance between centroids closest to the center
        '''
        # Calculate the distance between the centroids closest to the center (either side)
        left_cx = cx_array[cx_array < center_x]
        right_cx = cx_array[cx_array > center_x]
        if len(left_cx) == 0 or len(right_cx) == 0:
            #self.get_logger().warn("No centroids detected on one side of the center.")
            cross_msg.data = "none"
            self.cross_detect_pub.publish(cross_msg)
            return
        # Calculate the minimum distance between centroids on either side of the center
        min_distance = min(abs(left_cx[-1] - right_cx[0]), abs(right_cx[-1] - left_cx[0]))
        # Set a flexible threshold based on the minimum distance
        flexible_threshold_max = min_distance * 1.1
        flexible_threshold_min = min_distance * 0.9
        
        # Filter centroids based on the flexible threshold
        self.cx_filtered = cx_array[(cx_array > center_x - flexible_threshold_max) & (cx_array < center_x + flexible_threshold_max)]
        self.cy_filtered = cy_array[(cy_array > center_x - flexible_threshold_max) & (cy_array < center_x + flexible_threshold_max)]
           

        # Check number of centroids detected
        if len(self.cx_filtered) > 4:
            # More than 3 centroids detected, likely a crossing
            self.get_logger().info("Crossing detected!",throttle_duration_sec=0.5)
            # Set flag for crossing detection
            self.cross_in_fov = True
            if self.calculate_delta_y(self.cy_filtered) > self.dy_precheck_thresh:
                cross_msg.data = "approach_over"
                self.get_logger().info("Crossing approaching, over alignment threshold.",throttle_duration_sec=0.5)
                # Publish crossing message
                self.cross_detect_pub.publish(cross_msg)
                self.delta_y_pub.publish(Float32(data=self.calculate_delta_y(self.cy_filtered)))
            else:
                # Publish crossing message
                cross_msg.data = "approach"
                self.cross_detect_pub.publish(cross_msg)
        else:
            # Less than 3 centroids detected, not a crossing
            if self.cross_in_fov:
                # Set flag for close crossing detection
                self.get_logger().info("Crossing no longer in FOV.")
                self.cross_in_prox = True
                # Publish crossing message
                cross_msg.data = "xing"
                self.cross_detect_pub.publish(cross_msg)
            else:
                # No crossing detected
                self.get_logger().info("No crossing detected.", throttle_duration_sec=2.0)
                # Reset flags
                self.cross_in_prox = False
                self.cross_in_fov = False
                # Publish crossing message
                cross_msg.data = "none"
                self.cross_detect_pub.publish(cross_msg)
            self.cross_in_fov = False
            
            
    def calculate_delta_y(self, cy_array):
        '''
        Calculate the average delta y between centroids in the cy_array
        '''
        if len(cy_array) < 3:
            return 0.0
        
        # Calculate deltas
        deltas = np.diff(cy_array)
        # Calculate average delta
        avg_delta_y = np.mean(deltas)
        
        return avg_delta_y
    
    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('ROS time is active!')
        
def main(args=None):
    rclpy.init(args=args)
    node = LineCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
        
        
        
        
# End of file