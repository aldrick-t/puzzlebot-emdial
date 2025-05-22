''' 
img_capture.py

Image Capture Node
Node captures a series of images (determined by time or number of images).
General-use node for multiple applications.
Runs headless.
To visualize the camera input images, use the VisualMonitor node.
Originally designed to capture images for training a neural network.

aldrick-t
MAY 2025
'''

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor

class ImgCapture(Node):
    
    
    def __init__(self):
        super().__init__('img_capture')
        self.bridge = CvBridge()
        
        self.wait_for_ros_time()
        
        # Logger initialization 
        # Set logging level default to DEBUG for all nodes
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG)
        # Start logger
        self.logger = self.get_logger()
        
        #Init message
        self.logger.info("ImgCapture Starting Initialization...")

        # Parameters immutable after startup sequence
        # Video source (determined by launch mode)
        self.declare_parameter('camera_topic', 'video_source/raw')
        
        # Dynamic parameters
        # Image capture toggle
        self.declare_parameter('capture', False)
        # Image capture interval
        self.declare_parameter('interval', 0.5)
        # Image capture count
        self.declare_parameter('count', 30)
        # Image capture directory
        self.declare_parameter('directory', '/home/user/captured_images')
        # Image capture format
        self.declare_parameter('format', 'jpg')
        # Logging level
        self.declare_parameter('log_severity', 'INFO')
        
        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Subscriptions
        # Camera stream topic
        self.create_subscription(Image, self.get_parameter('camera_topic').value, self.camera_callback, 10)
        # Publishers
        # No publishers needed for this node
        
        # Timer for image capture
        self.timer = None
        # Image capture variables
        self.capture = False
        self.interval = 1.0
        self.count = 10
        self.directory = '/home/user/captured_images'
        self.format = 'jpg'
        self.image_count = 0
        self.image = None
        
        # Running log message
        self.logger.info("ImgCapture Initialized!")
        
    def timer_cb(self):
        pass
        
    def camera_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        if self.capture:
            pass
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'capture':
                self.capture = param.value
                self.get_logger().info(f"Image capture {param.value}")
                
            elif param.name == 'interval':
                self.interval = param.value
                self.get_logger().info(f"Image capture interval {param.value}")
                
            elif param.name == 'count':
                self.count = param.value
                self.get_logger().info(f"Image capture count {param.value}")
                
            elif param.name == 'directory':
                self.directory = param.value
                self.get_logger().info(f"Image capture directory {param.value}")
                
            elif param.name == 'format':
                self.format = param.value
                self.get_logger().info(f"Image capture format {param.value}")
                
            elif param.name == 'log_severity':
                severity = LoggingSeverity[param.value.upper()]
                rclpy.logging.get_logger('rclpy').set_level(severity)
                self.get_logger().set_level(severity)
                self.get_logger().info(f"Logging level set to {param.value}")

        return SetParametersResult(successful=True)
    
            
        
    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('ROS time is active!')
    
    
