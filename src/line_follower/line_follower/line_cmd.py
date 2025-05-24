'''
line_cmd.py

Line Command Node
Node subscribes to LineRecogni topics and publishes commands based on line detection.

aldrick-t
MAY 2025
'''

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
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
        
        # Header vars
        self.img = None
        
        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Subscriptions
        # Image stream topic
        self.create_subscription(Image, 'prox_overlay', self.img_cb, 10)
        # Data stream topic
        self.create_subscription(Int32, 'line_recogni', self.line_cmd_cb, 10)
        self.create_subscription(Int32, 'line_recogni_mid', self.line_cmd_cb, 10)
        
        # Publishers
        # Line following interpreted command
        self.line_cmd_pub = self.create_publisher(Float32, 'line_cmd', 10)
        # Image publishers
        self.thresh_overlay_pub = self.create_publisher(Image, 'thresh_overlay', 10)
        
        # Running message
        self.logger.info("LineRecogni Initialized!")
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'log_severity':
                severity = getattr(LoggingSeverity, param.value)
                self.get_logger().set_level(severity)
                rclpy.logging.get_logger('rclpy').set_level(severity)
                self.get_logger().info(f"Log severity set to {param.value}")
        
        return SetParametersResult(successful=True)
    
    def img_cb(self, msg):
        '''
        Callback function for camera image topic
        '''
        # Convert ROS Image message to OpenCV image
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Process source image data
        self.width, self.center_x, self.null_thresh_l, self.null_thresh_r, img_overlay = self.process_source_image(self.img)
        # Publish thresh overlay image
        self.thresh_overlay_pub.publish(self.bridge.cv2_to_imgmsg(img_overlay, encoding='bgr8'))
        
        
    def line_cmd_cb(self, msg):
        '''
        Callback function for line command topic
        '''
        # Gather data from source image processing
        process_img_data = [self.img, self.width, self.center_x, self.null_thresh_l, self.null_thresh_r]
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
            self.get_logger().debug(f"AAAAAAAAAAAAAAAAAAAA: {cmd_msg}", throttle_duration_sec=1.0)
            cmd_msg += int(cmd_msg)
            cmd_msg = -1.0 + cmd_msg
            cmd_msg *= -1.0
        #self.get_logger().debug("SEXOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO", throttle_duration_sec=1.0)
        cmd_msg = Float32()
        self.line_cmd_pub.publish(cmd_msg)
        
        
        
    def process_source_image(self, img):
        '''
        Process the source image to extract information
        '''
        # Get image dimensions
        height, width = img.shape[:2]
        # Calculate center of the image
        center_x = width // 2
        # Calculate null thresholds
        # Define null thresholds as a percentage of the image width
        # null_thresh_l: % from the left edge of the image
        # null_thresh_r: % from the left edge of the image (or % from the right edge)
        null_thresh_l = int(center_x * 0.98)
        null_thresh_r = int(center_x * 1.02)
        
        # Add to overlay image
        cv2.line(img, (null_thresh_l, 0), (null_thresh_l, height), (255, 0, 100), 3)
        cv2.line(img, (null_thresh_r, 0), (null_thresh_r, height), (255, 0, 100), 3)
        
        return width, center_x, null_thresh_l, null_thresh_r, img
        
        
    def process_line_cmd(self, process_img_data, line_recogni_data):
        '''
        Process the line command based on collected line recognition data
        Outputs cmd value in a range from -1 to 1 based on line position relative to image center
        Considers set thresholds for null zones (set cmd to 0)
        '''
        # Initialize command message
        cmd_msg = Float32()
        # Extract data from process image
        img, width, center_x, null_thresh_l, null_thresh_r = process_img_data
        # Extract line recognition data
        line_recogni = line_recogni_data
        # Calculate command value based on line position
        if line_recogni > null_thresh_l and line_recogni < null_thresh_r:
            # Line is within the null zone
            cmd_msg.data = 0.0
        elif line_recogni < null_thresh_l:
            # Line is to the left of the null zone
            cmd_msg.data = -1.0 * ((center_x - line_recogni) / center_x) + null_thresh_l
        elif line_recogni > null_thresh_r:
            # Line is to the right of the null zone
            cmd_msg.data = 1.0 * ((line_recogni - center_x) / center_x) - null_thresh_r
            
        # Log command value
        self.get_logger().debug(f"Line command: {cmd_msg.data}", throttle_duration_sec=20.0)
        
        return cmd_msg
        
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