''' 
line_recogni.py
Refactored for puzzlebot_emdial package.

Line Recognition Node
Node subscribes to camera stream topic and applies line recognition algorithm.
Publishes overlay image stream for visualization and debugging.
Node for line following applications.
Runs headless.

aldrick-t
JUN 2025
'''

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Int32 
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
        self.declare_parameter('midfield_range', 0.55)  # Midfield range as a fraction of the image height
        self.declare_parameter('proximal_range', 0.07)  # Proximal range as a fraction of the image height
        self.declare_parameter('bin_thresh', 80) # Threshold for the binary cutoff thresh
        # Personalization parameters
        self.declare_parameter('resolution_factor', 1)  # Factor to  scale resolution of the overlay 
        # Contour area parameters
        self.declare_parameter('min_cnt_area', 5)  # Minimum contour area for mid-range line detection
        self.declare_parameter('max_cnt_area', 700)  # Maximum contour area for mid-range line detection
        self.declare_parameter('cnt_area_thresh_mode', 'fixed')  # Mode for contour area thresholding
        
        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Parameter variables
        self.res_factor = self.get_parameter('resolution_factor').value
        self.midfield_range = 1 - self.get_parameter('midfield_range').value
        self.proximal_range = 1 - self.get_parameter('proximal_range').value
        self.bin_thresh = self.get_parameter('bin_thresh').value
        self.min_cnt_area = self.get_parameter('min_cnt_area').value
        self.max_cnt_area = self.get_parameter('max_cnt_area').value
        self.cnt_area_thresh_mode = self.get_parameter('cnt_area_thresh_mode').value
        
        # Program variables
        self.prox_c_height = None  # Proximal cropped image height
        self.prox_c_width = None   # Proximal cropped image width
        self.mid_c_height = None   # Midrange cropped image height
        self.mid_c_width = None    # Midrange cropped image width
        
        self.viewfield_dim_array = Int32MultiArray()
        self.viewfield_dim_array.data = [0,0, 0,0, 0,0]  # Initialize with zero dimensions
        
        # Subscriptions
        self.create_subscription(Image, self.get_parameter('camera_topic').value, self.img_cb, 10)
        
        # Publishers
        # Image publishers
        self.prox_overlay_pub = self.create_publisher(Image, 'lr_overlay', 10)
        # Data publishers
        self.line_recogni_prox_pub = self.create_publisher(Int32, 'line_recogni', 10)
        self.line_recogni_mid_cx_pub = self.create_publisher(Int32MultiArray, 'line_recogni_mid_cx', 10)
        self.line_recogni_mid_cy_pub = self.create_publisher(Int32MultiArray, 'line_recogni_mid_cy', 10)
        # Viewfield dimensions publisher
        self.viewfield_dim_pub = self.create_publisher(Int32MultiArray, 'viewfield_dim', 10)
        
        # Running message
        self.logger.info("LineRecogni Initialized!")
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'log_severity':
                severity = getattr(LoggingSeverity, param.value)
                self.get_logger().set_level(severity)
                rclpy.logging.get_logger('rclpy').set_level(severity)
                self.get_logger().info(f"Log severity set to {param.value}")
            elif param.name == 'resolution_factor':
                self.res_factor = param.value
                self.get_logger().info(f"Resolution factor set to {self.res_factor}")
            elif param.name == 'midfield_range':
                self.midfield_range = param.value
                self.get_logger().info(f"Midfield range set to {self.midfield_range}")
            elif param.name == 'proximal_range':
                self.proximal_range = param.value
                self.get_logger().info(f"Proximal range set to {self.proximal_range}")
            elif param.name == 'bin_thresh':
                self.bin_thresh = param.value
                self.get_logger().info(f"Bin. threshold set to {self.bin_thresh}")
            elif param.name == 'min_cnt_area':
                self.min_cnt_area = param.value
                self.get_logger().info(f"Min. contour area set to {self.min_cnt_area}")
            elif param.name == 'max_cnt_area':
                self.max_cnt_area = param.value
                self.get_logger().info(f"Max. contour area set to {self.max_cnt_area}")
            elif param.name == 'cnt_area_thresh_mode':
                if param.value not in ['fixed', 'dynamic']:
                    self.get_logger().error(f"Invalid contour area threshold mode: {param.value}. Must be 'fixed' or 'dynamic'.")
                    return SetParametersResult(successful=False, reason="Invalid contour area threshold mode")
                self.cnt_area_thresh_mode = param.value
                self.get_logger().info(f"Contour area threshold mode set to {self.cnt_area_thresh_mode}")
        
        return SetParametersResult(successful=True)
    
    def img_cb(self, msg):
        ''' 
        Callback function for grayscale processed image topic
        '''
        prox_cx_msgInt = Int32()
        
        mid_all_cx_msgIntArray = Int32MultiArray()
        mid_all_cy_msgIntArray = Int32MultiArray()
        
        # Convert ROS image to OpenCV format
        cv_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # CV preprocessing
        cv_preprocessed_fullframe = self.preprocess_fullframe(cv_raw)
        
        overlay_image = cv_preprocessed_fullframe.copy()
        overlay_image = cv2.cvtColor(overlay_image, cv2.COLOR_GRAY2BGR)
        
        prox_image, prox_overlay_image, prox_centroid_x = self.process_proximal_line(cv_preprocessed_fullframe, overlay_image)
        
        overlay_image = prox_overlay_image.copy()
        
        mid_image, mid_overlay_image, mid_centroid_x, mid_all_cx, mid_all_cy = self.process_midrange_line(cv_preprocessed_fullframe, overlay_image)
        
        # Calculate viewfield dimensions for all images
        self.viewfield_dim_array.data = [
            cv_raw.shape[0],  # Original image height
            cv_raw.shape[1],  # Original image width
            prox_image.shape[0],  # Proximal cropped image height
            prox_image.shape[1],  # Proximal cropped image width
            mid_image.shape[0],  # Mid-range cropped image height
            mid_image.shape[1]   # Mid-range cropped image width
        ]
        # publish viewfield dims
        self.viewfield_dim_pub.publish(self.viewfield_dim_array)
        
        # Convert processed images back to ROS format
        mid_overlay_image = self.bridge.cv2_to_imgmsg(overlay_image, encoding='bgr8')
        
        # Publish the processed images
        self.prox_overlay_pub.publish(mid_overlay_image)
        
        # Publish the centroid x coordinate
        if prox_centroid_x is None:
            self.get_logger().debug("No line detected; publishing None", throttle_duration_sec=2.0)
            prox_cx_msgInt.data = 1500  # Use -1 to indicate no line detected
        else:
            prox_cx_msgInt.data = int(prox_centroid_x)
        self.line_recogni_prox_pub.publish(prox_cx_msgInt)
        
        # Publish the mid-range centroid x coordinates
        if mid_all_cx is None:
            #self.get_logger().debug("No mid-range line detected; publishing None", throttle_duration_sec=2.0)
            mid_all_cx_msgIntArray.data = []
        else:
            #self.get_logger().debug(f"Mid-range line detected; centroids: {mid_all_centroid_x}", throttle_duration_sec=2.0)
            mid_all_cx_msgIntArray.data = [int(x) for x in mid_all_cx]
        self.line_recogni_mid_cx_pub.publish(mid_all_cx_msgIntArray)
        # Publish the mid-range centroid y coordinates
        if mid_all_cy is None:
            #self.get_logger().debug("No mid-range line detected; publishing None", throttle_duration_sec=2.0)
            mid_all_cy_msgIntArray.data = []
        else:
            #self.get_logger().debug(f"Mid-range line detected; centroids: {mid_all_centroid_y}", throttle_duration_sec=2.0)
            mid_all_cy_msgIntArray.data = [int(y) for y in mid_all_cy]
        self.line_recogni_mid_cy_pub.publish(mid_all_cy_msgIntArray)
            
        
    def preprocess_fullframe(self, image):
        '''
        Preprocess the full frame image for line detection.
        Applies Gaussian blur, grayscale conversion, and fixed inverse binary thresholding.
        '''
        # Convert to grayscale
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        image = cv2.GaussianBlur(image, (5, 5), 0)
        
        # Fixed binary thresholding (inverted for contour detection)
        # Parameters: cv2.threshold(src, thresh, maxval, type)
        _, image = cv2.threshold(image, self.bin_thresh, 255, cv2.THRESH_BINARY_INV)
        
        # Morphological Ops
        # Apply morphological operations to remove noise
        kernel5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        kernel3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        kernel2 = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        image = cv2.morphologyEx(image, cv2.MORPH_ERODE, kernel3, iterations=1)
        image = cv2.morphologyEx(image, cv2.MORPH_ERODE, kernel2, iterations=1)
        image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel5, iterations=1)
        #image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel2)
        # image = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        
        
        return image
 
    def process_proximal_line(self, image, overlay_image):
        '''
        Process the image to detect prox range line.
        Calculates the centroid of the determined contour (centerline in proximal range).
        '''
        # Get original image dimensions
        height, width = image.shape[:2]
        # Get overlay image dimensions
        o_height, o_width = overlay_image.shape[:2]
        # Crop the image (crop from top to bottom)
        crop_height = int(height * self.proximal_range)
        image = image[crop_height:, :] 
        
        # Cropped image dimensions
        c_height, c_width = image.shape[:2]
        self.prox_c_height = c_height
        self.prox_c_width = c_width
        
        # Find contours
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Condition to check if contours are found
        if not contours:
            self.get_logger().debug("No contours found", throttle_duration_sec=5.0)
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
        
        # Convert to BGR for colroful overlay
            
        # Draw all contours with offset to account for cropping
        offset_y = crop_height  # vertical offset due to cropping the image from crop_height to bottom
        # Create adjusted contours by adding offset to y-coordinates
        adjusted_contours = [cnt.copy() for cnt in contours]
        for cnt in adjusted_contours:
            cnt[:, :, 1] += offset_y
        
        # Adjust the largest contour as well
        adjusted_max_contour = max_contour.copy()
        adjusted_max_contour[:, :, 1] += offset_y

        # Draw all adjusted contours
        cv2.drawContours(overlay_image, adjusted_contours, -1, (0, 200, 255), self.res_factor)
        # Draw the adjusted largest contour
        cv2.drawContours(overlay_image, [adjusted_max_contour], -1, (0, 255, 0), self.res_factor)
        
        # Draw the centroid for max contour if it exists
        if cx is not None and cy is not None:
            # Draw the centroid
            # Parameters: cv2.circle(img, center, radius, color, thickness)
            cv2.circle(overlay_image, (cx, (crop_height+cy)), 5, (0, 0, 255), -1)
            #Draw vertical line following centroid
            cv2.line(overlay_image, (cx, 0), (cx, height), (0, 0, 255), self.res_factor)
            
        #Draw fixed vertical line at center of image
        # Parameters: cv2.line(img, pt1, pt2, color, thickness)
        cv2.line(overlay_image, (int(c_width/2), 0), (int(c_width/2), o_height), (255, 100, 0), self.res_factor)
        
        # Draw horizontal line to indicate crop limit
        cv2.line(overlay_image, (0, crop_height), (o_width, crop_height), (255, 0, 0), self.res_factor)
        
        return image, overlay_image, cx
    
    def process_midrange_line(self, image, overlay_image):
        '''
        Process the image to detect mid-range line.
        Applies Crop, Gaussian blur,
        '''
        # Get original image dimensions
        height, width = image.shape[:2]
        # Get overlay image dimensions
        o_height, o_width = overlay_image.shape[:2]
        # Crop the image to focus on the mid-range
        crop_height_top = int(height * self.midfield_range)
        crop_height_bottom = int(height * 1.00)
        # Crop Sides
        # crop_width_left
        image = image[crop_height_top:crop_height_bottom, :]
        
        # Cropped image dimensions
        c_height, c_width = image.shape[:2]
        
        # Find contours
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Condition to check if contours are found
        if not contours:
            self.get_logger().debug("No contours found", throttle_duration_sec=5.0)
            return image, overlay_image, None, None, None
        else:
            self.get_logger().debug("No valid moments; no line detected", throttle_duration_sec=5.0)
            cx, cy = None, None
            
        # Find the largest contour
        # Parameters: max(contours, key=cv2.contourArea)
        max_contour = max(contours, key=cv2.contourArea)
        
        # Sort contours by area
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        
        if self.cnt_area_thresh_mode == 'dynamic':
            # Dynamic contour area thresholding
            # Calculate dynamic thresholds based on the largest contour area
            max_area = cv2.contourArea(max_contour)
            min_area = max(self.min_cnt_area, int(max_area * 0.50))  # Min area is a % of the largest contour area
            moments = [cv2.moments(cnt) 
                    for cnt in contours 
                    if cv2.contourArea(cnt) > min_area 
                    and cv2.contourArea(cnt) < max_area
                    ]
            
        elif self.cnt_area_thresh_mode == 'fixed':
            # Compute moments for all contours within area threshold (min, max)
            moments = [cv2.moments(cnt) 
                    for cnt in contours 
                    if cv2.contourArea(cnt) > self.min_cnt_area 
                    and cv2.contourArea(cnt) < self.max_cnt_area
                    ]
            
        # Filter out contours with zero area
        valid_moments = [m for m in moments if m["m00"] != 0]
        if not valid_moments:
            self.get_logger().debug("No valid moments; no line detected", throttle_duration_sec=5.0)
            return image, overlay_image, None, None, None
        
        # Calculate centroid for all valid contours
        centroids = [(int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])) for m in valid_moments]
        
        # Calculate centroid for the largest contour
        max_moment = cv2.moments(max_contour)
        if max_moment["m00"] != 0:
            cx = int(max_moment["m10"] / max_moment["m00"])
            cy = int(max_moment["m01"] / max_moment["m00"])
            self.get_logger().debug(f"Centroid: ({cx}, {cy})", throttle_duration_sec=10.0)
        else:
            self.get_logger().debug("No contour found", throttle_duration_sec=5.0)
        
        # Calculate average centroid if multiple valid contours exist
        if len(centroids) > 1:
            avg_cx = int(np.mean([c[0] for c in centroids]))
            avg_cy = int(np.mean([c[1] for c in centroids]))
            self.get_logger().debug(f"Average Centroid: ({avg_cx}, {avg_cy})", throttle_duration_sec=10.0)
            cx, cy = avg_cx, avg_cy
        else:
            self.get_logger().debug("Only one valid contour; using its centroid", throttle_duration_sec=5.0)
        
        # Parse all centroid coordinates into a list
        # Only valid centroids are considered
        # Isolating only the x-coordinates for line following
        # If no valid contours, cx will be None
        cx_all = [c[0] for c in centroids if c[0] is not None]
        if not cx_all:
            return image, overlay_image, None, None, None
        
        # Parse all y-coordinates for centroids
        cy_all = [c[1] for c in centroids if c[1] is not None]
        if not cy_all:
            return image, overlay_image, None, None, None
        
        # Draw Overlay
        
        # Draw all contours with offset to account for cropping
        offset_y = crop_height_top  # vertical offset due to cropping the image from crop_height to bottom
        # Create adjusted contours by adding offset to y-coordinates
        adjusted_contours = [cnt.copy() for cnt in contours]
        for cnt in adjusted_contours:
            cnt[:, :, 1] += offset_y
        
        # Adjust the largest contour as well
        adjusted_max_contour = max_contour.copy()
        adjusted_max_contour[:, :, 1] += offset_y

        # Draw all adjusted contours
        cv2.drawContours(overlay_image, adjusted_contours, -1, (0, 200, 255), self.res_factor)
        # Draw the adjusted largest contour
        cv2.drawContours(overlay_image, [adjusted_max_contour], -1, (0, 255, 0), self.res_factor)
        
        # Draw all centroids for valid contours
        for cx, cy in centroids:
            # Draw the centroid
            cv2.circle(overlay_image, (cx, (crop_height_top+cy)), 5, (0, 200, 255), -1)
            # Draw vertical line following centroid
            cv2.line(overlay_image, (cx, 0), (cx, height), (0, 200, 255), self.res_factor)
        
        # Draw the centroid for max contour if it exists
        if cx is not None and cy is not None:
            # Draw the centroid
            cv2.circle(overlay_image, (cx, (crop_height_top+cy)), 5, (0, 0, 255), -1)
            # Draw vertical line following centroid
            cv2.line(overlay_image, (cx, 0), (cx, height), (0, 0, 255), self.res_factor)
        
        # Draw fixed vertical line at center of image
        cv2.line(overlay_image, (int(c_width/2), 0), (int(c_width/2), o_height), (255, 100, 0), self.res_factor)
        
        # Draw horizontal lines to indicate crop limits
        cv2.line(overlay_image, (0, crop_height_top), (o_width, crop_height_top), (255, 0, 0), self.res_factor)
        cv2.line(overlay_image, (0, crop_height_bottom), (o_width, crop_height_bottom), (255, 0, 0), self.res_factor)
        
        return image, overlay_image, cx, cx_all, cy_all
    
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