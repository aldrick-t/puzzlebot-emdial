'''
img_capture.py

Image Capture Node
Node captures a series of images (determined by time or number of images).
General-use node for multiple applications.
Runs headless.
To visualize the camera input images, use the VisualMonitor node.
Originally designed to capture images for training a neural network.
Use onboard camera for best results.

aldrick-t
MAY 2025
'''

import os
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from rclpy.logging import LoggingSeverity
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImgCapture(Node):
    # Original: Initialize ImgCapture node
    def __init__(self):
        super().__init__('img_capture')
        self.bridge = CvBridge()

        self.wait_for_ros_time()  # block until ROS time is valid

        # Logger initialization 
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.logger = self.get_logger()
        self.logger.info("ImgCapture Starting Initialization...")

        # Declare ROS parameters 
        self.declare_parameter('camera_topic', 'video_source/raw')
        self.declare_parameter('capture', False)
        self.declare_parameter('interval', 0.1)
        self.declare_parameter('count', 30)
        self.declare_parameter('directory', '/home/atad/captured_images')
        self.declare_parameter('format', 'jpg')
        self.declare_parameter('log_severity', 'INFO')
        self.declare_parameter('img_author', 'emdial')
        self.declare_parameter('img_description', 'tl-red')

        # Parameter change callback 
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Load initial values
        self.camera_topic = self.get_parameter('camera_topic').value
        self.capture = self.get_parameter('capture').value
        self.interval = self.get_parameter('interval').value
        self.count = self.get_parameter('count').value
        self.directory = self.get_parameter('directory').value
        self.img_format = self.get_parameter('format').value
        self.img_author = self.get_parameter('img_author').value
        self.img_description = self.get_parameter('img_description').value
        self.set_log_level(self.get_parameter('log_severity').value)

        # Ensure save directory exists 
        os.makedirs(self.directory, exist_ok=True)

        # Internal state
        self.img = None
        self.img_count = 0

        # Subscriptions and timer setup
        self.subscription = None
        self.img_timer = None
        self.setup_subscription()   
        self.setup_timer()          

        self.logger.info('ImgCapture Initialized!') 

    #Helper to (re)create camera subscription based on current topic
    def setup_subscription(self):
        if self.subscription:
            self.subscription.destroy()
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.camera_callback,
            10
        )
        self.get_logger().info(f"Subscribed to camera topic: {self.camera_topic}")

    #Helper to (re)create timer when interval changes
    def setup_timer(self):
        if self.img_timer:
            self.img_timer.cancel()
        self.img_timer = self.create_timer(
            self.interval,
            self.capture_timer_cb
        )
        self.get_logger().info(f"Capture timer interval set to: {self.interval} seconds")

    #Renamed from timer_cb to capture_timer_cb, now skips when not capturing
    def capture_timer_cb(self):
        '''
        Timer callback for image capture
        '''
        if not self.capture:
            return  #Skip if capture disabled
        if self.img is None:
            self.get_logger().warning("No image received yet; skipping capture")  # Added
            return

        # Save image
        filename = f"{self.directory}/{self.img_description}_{self.img_count}_{self.img_author}.{self.img_format}"
        cv2.imwrite(filename, self.img)
        self.get_logger().info(f"Captured image: {filename}, count: {self.img_count}")
        self.img_count += 1

        #Stop after count reached
        if self.img_count >= self.count:
            self.capture = False
            self.get_logger().info("Image capture completed.")

    # Camera callback
    def camera_callback(self, msg):
        '''
        Camera Callback fetches latest video frame
        '''
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().debug("Called CameraCB... Received Image...")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")  # Added error handling

    #Parameter callback handles topic/interval resets
    def parameter_callback(self, params):
        results = []
        for param in params:
            if param.name == 'camera_topic':
                self.camera_topic = param.value
                self.setup_subscription() 
                self.get_logger().info(f"Camera topic set to: {param.value}")
            elif param.name == 'interval':
                self.interval = float(param.value)
                self.setup_timer() 
                self.get_logger().info(f"Interval set to: {param.value}")
            elif param.name == 'capture':
                self.capture = bool(param.value)
                if self.capture:
                    self.img_count = 0  
                self.get_logger().info(f"Capture toggled to: {param.value}")
            elif param.name == 'count':
                self.count = int(param.value)
                self.get_logger().info(f"Count set to: {param.value}")
            elif param.name == 'directory':
                self.directory = param.value
                os.makedirs(self.directory, exist_ok=True) 
                self.get_logger().info(f"Directory set to: {param.value}")
            elif param.name == 'format':
                self.img_format = param.value
                self.get_logger().info(f"Format set to: {param.value}")
            elif param.name == 'img_author':
                self.img_author = param.value
                self.get_logger().info(f"Author set to: {param.value}")
            elif param.name == 'img_description':
                self.img_description = param.value
                self.get_logger().info(f"Description set to: {param.value}")
            elif param.name == 'log_severity':
                self.set_log_level(param.value)
            else:
                continue
        return SetParametersResult(successful=True)

    def wait_for_ros_time(self):
        '''
        Waits until ROS time is non-zero
        '''
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('ROS time is active!')

    def set_log_level(self, level_str: str):
        try:
            sev = LoggingSeverity[level_str.upper()]
            self.get_logger().set_level(sev)
            rclpy.logging.get_logger('rclpy').set_level(sev)
            self.get_logger().info(f"Log severity set to: {level_str}")
        except KeyError:
            self.get_logger().warning(f"Invalid log severity: {level_str}")


def main(args=None):
    rclpy.init(args=args)
    node = ImgCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
