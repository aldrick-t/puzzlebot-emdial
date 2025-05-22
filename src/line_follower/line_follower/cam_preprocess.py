''' 
cam_preprocess.py

Camera Preprocess Node
Node subscribes to camera stream topic and applies basic preprocessing.
Publishes multiple processed image streams for later multipurpose use.
General-use node for multiple applications.
Runs headless.

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

class CamPreprocess(Node):
    '''
    Camera Preprocess Node
    Subscriptions:
    - /"undefined raw camera stream": Camera stream topic
    Publishers:
    - /hsv_processed: HSV preprocessed image
    - /gray_processed: Grayscale preprocessed image
    '''
    def __init__(self):
        super().__init__('cam_preprocess')
        self.bridge = CvBridge()
        
        self.wait_for_ros_time()
        
        # Logger initialization 
        # Set logging level default to DEBUG for all nodes
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG)
        # Start logger
        self.logger = self.get_logger()
        
        #Init message
        self.logger.info("CamPreprocess Starting Initialization...")

        # Parameters immutable after startup sequence
        # Video source (determined by launch mode)
        self.declare_parameter('camera_topic', 'video_source/raw')

        # Dynamic parameters
        # HSV out toggle
        self.declare_parameter('hsv_out', False)
        # Grayscale out toggle
        self.declare_parameter('gray_out', True)
        # Logging level
        self.declare_parameter('log_severity', 'INFO')
        
        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Subscriptions
        # Camera stream topic
        self.create_subscription(Image, self.get_parameter('camera_topic').value, self.camera_callback, 10)
        
        # Publishers
        # HSV preprocessed image
        self.hsv_pub = self.create_publisher(Image, 'hsv_processed', 10)
        # Grayscale preprocessed image
        self.gray_pub = self.create_publisher(Image, 'gray_processed', 10)
        
        # Running message
        self.logger.info("CamPreprocess Initialized!")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'hsv_out':
                self.hsv_out = param.value
                self.get_logger().info(f"Grayscale image stream {param.value}")
                
            elif param.name == 'gray_out':
                self.gray_out = param.value
                self.get_logger().info(f"Grayscale image stream {param.value}")
            
            elif param.name == 'log_severity':
                severity = getattr(LoggingSeverity, param.value)
                self.get_logger().set_level(severity)
                rclpy.logging.get_logger('rclpy').set_level(severity)
                self.get_logger().info(f"Log severity set to {param.value}")
    
        return SetParametersResult(successful=True)        
    
    def camera_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Processing for GRAYSCALE
            #Conditional if enabled
            if self.get_parameter('gray_out').value:
                gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                gray_msg = self.bridge.cv2_to_imgmsg(gray_image, encoding='mono8')
                self.gray_pub.publish(gray_msg)
                # self.get_logger().debug("Gray pub... ")
            
            # Processing for HSV
            # Conditional if enabled
            if self.get_parameter('hsv_out').value:
                hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                hsv_msg = self.bridge.cv2_to_imgmsg(hsv_image, encoding='bgr8')
                self.hsv_pub.publish(hsv_msg)
                # self.get_logger().debug("HSV pub...")
                
        except Exception as e:
            self.get_logger().error(f"camera_callback error: {e}")
            
    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('ROS time is active!')
            
def main(args=None):
    rclpy.init(args=args)
    node = CamPreprocess()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()