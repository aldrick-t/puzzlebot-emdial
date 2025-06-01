''' 
line_recogni.py
Refactored for puzzlebot_emdial package.

Line Recognition Node
Node subscribes to camera stream topic and applies line recognition algorithm.
Publishes overlay image stream for visualization and debugging.
Node for line following applications.
Runs headless.

aldrick-t
MAY 2025
'''

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import SetParametersResult


class LineRecogni(Node):
    '''
    Line Recognition Node
    Subscriptions:
    - /gray_processed: Grayscale preprocessed image
    
    Publishers:
    - /line_overlay: Overlay image with detected lines
    '''
    def __init__(self):
        super().__init__('line_recogni')
        self.bridge = CvBridge()
        
        self.wait_for_ros_time()
        
        # Logger initialization 
        # Set logging level default to DEBUG for all nodes
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG)
        # Start logger
        self.logger = self.get_logger()
        
        #Init message
        self.logger.info("LineRecogni Starting Initialization...")
    
        # Parameters immutable after startup sequence
        # Video source (determined by launch mode)
        self.declare_parameter('camera_topic', 'video_source/raw')
        
        # Dynamic parameters
        # Logging level
        self.declare_parameter('log_severity', 'DEBUG')
        # Line detection parameters
        # None
        
        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Subscriptions
        self.create_subscription(Image, self.get_parameter('camera_topic'), self.img_cb, 10)
        
        # Publishers
        # Image publishers
        self.prox_overlay_pub = self.create_publisher(Image, 'lr_overlay', 10)
        # Data publishers
        self.line_recogni_prox_pub = self.create_publisher(Int32, 'line_recogni', 10)
        self.line_recogni_mid_pub = self.create_publisher(Int32, 'line_recogni_mid', 10)
        
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
        Callback function for grayscale processed image topic
        '''
        msgInt = Int32()
        
        # Convert ROS image to OpenCV format
        cv_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # CV preprocessing
        cv_preprocessed_fullframe = self.preprocess_fullframe(cv_raw)
        
        prox_image, prox_overlay_image, prox_centroid_x = self.process_proximal_line(cv_preprocessed_fullframe)
        mid_image, mid_overlay_image, mid_centroid_x = self.process_midrange_line(cv_preprocessed_fullframe)
        
        # Convert processed images back to ROS format
        prox_overlay_ros_image = self.bridge.cv2_to_imgmsg(prox_overlay_image, encoding='bgr8')
        
        # Publish the processed images
        self.prox_overlay_pub.publish(prox_overlay_ros_image)
        
        # Publish the centroid x coordinate
        if prox_centroid_x is None:
            self.get_logger().debug("No line detected; publishing None", throttle_duration_sec=2.0)
            # Using -1 here to indicate lack of detection. Adjust as needed.
            # msgInt.data = -1  #Commented out to avoid confusion with valid data
        else:
            msgInt.data = int(prox_centroid_x)
        self.line_recogni_prox_pub.publish(msgInt)
        
    def preprocess_fullframe(self, image):
        '''
        Preprocess the full frame image for line detection.
        Applies Gaussian blur, grayscale conversion, and fixed binary thresholding.
        '''
        # Convert to grayscale
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        image = cv2.GaussianBlur(image, (5, 5), 0)
        
        # Fixed binary thresholding (inverted for contour detection)
        # Parameters: cv2.threshold(src, thresh, maxval, type)
        _, image = cv2.threshold(image, 80, 255, cv2.THRESH_BINARY_INV)
        
        return image
 
    def process_proximal_line(self, image):
        '''
        Process the image to detect prox range line.
        Calculates the centroid of the determined contour (centerline in proximal range).
        '''
        
        overlay_image = image.copy()
        
        # Crop the image (crop from top to bottom)
        height, width = image.shape[:2]
        crop_height = int(height * 0.85)
        image = image[crop_height:, :] 
        
        # Cropped image dimensions
        c_height, c_width = image.shape[:2]
        
        # Find contours
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Condition to check if contours are found
        if not contours:
            self.get_logger().debug("No contours found", throttle_duration_sec=5.0)
            overlay_image = cv2.cvtColor(overlay_image, cv2.COLOR_GRAY2BGR)
            return image, overlay_image, None
        else:
            self.get_logger().debug("No valid moments; no line detected", throttle_duration_sec=5.0)
            cx, cy = None, None
            
        # Find the largest contour
        # Parameters: max(contours, key=cv2.contourArea)
        max_contour = max(contours, key=cv2.contourArea)

        # Compute image moments
        moments = cv2.moments(max_contour)

        # Calculate centroid (cx, cy)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            self.get_logger().debug(f"Centroid: ({cx}, {cy})", throttle_duration_sec=10.0)
        else:
            self.get_logger().debug("No contour found", throttle_duration_sec=5.0)
            
        
        #Draw Overlay
        
        #Copy processed cropped image
        overlay_image = image.copy()
        # Convert to BGR for colroful overlay
        overlay_image = cv2.cvtColor(overlay_image, cv2.COLOR_GRAY2BGR)
            
        # Draw the contour and centroid on the image
        # Parameters: cv2.drawContours(img, contours, contourIdx, color, thickness)
        cv2.drawContours(overlay_image, [max_contour], -1, (0, 255, 0), 2)
        
        if cx is not None and cy is not None:
            # Draw the centroid
            # Parameters: cv2.circle(img, center, radius, color, thickness)
            cv2.circle(overlay_image, (cx, cy), 5, (0, 0, 255), -1)
            #Draw vertical line following centroid
            cv2.line(overlay_image, (cx, 0), (cx, height), (0, 0, 255), 2)
            
        #Draw fixed vertical line at center of image
        # Parameters: cv2.line(img, pt1, pt2, color, thickness)
        cv2.line(overlay_image, (int(c_width/2), 0), (int(c_width/2), c_height), (255, 100, 0), 2)
        
        
        
        return image, overlay_image, cx
    
    def process_midrange_line(self, image):
        '''
        Process the image to detect mid-range line.
        Applies Crop, Gaussian blur,
        '''
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        overlay_image = image.copy()
        
       
    
    
    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('ROS time is active!')
    
    
def main(args=None):
    rclpy.init(args=args)
    node = LineRecogni()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()