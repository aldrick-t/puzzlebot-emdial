import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
from rclpy.logging import LoggingSeverity


class CVExample(Node):
    def __init__(self): 
        super().__init__('color_detector')
        self.bridge = CvBridge()
        
        # DEBUG logging 
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.get_logger().debug('Initializing ColorDetectorNode!')

        self.sub = self.create_subscription(Image, 'camera/image_raw', self.camera_callback, 10) # camera || ideo_source/raw
        self.pub = self.create_publisher(Image, 'processed_img', 10) # Publisher for processed image
        self.color_pub = self.create_publisher(String, 'traffic_light_color', 10)  # New publisher for traffic light color

        self.image_received_flag = False
        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)
        self.get_logger().info('ros_color_tracker Node started')

    def camera_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform color detection
            color_detected, processed_img = self.detect_traffic_light_color(cv_img)

            # Publish processed image
            self.pub.publish(self.bridge.cv2_to_imgmsg(processed_img, 'bgr8'))

            # Publish detected color (if any)
            if color_detected:
                self.color_pub.publish(String(data=color_detected))
                self.get_logger().info(f'Detected color: {color_detected}')


            self.image_received_flag = True
        except Exception as e:
            self.get_logger().info(f'Failed to process image: {str(e)}')

    def detect_traffic_light_color(self, img):
        # Resize to make processing faster (optional)
        img = cv2.resize(img, (320, 240))  # or (160, 120)

        # Apply Gaussian Blur to reduce noise
        blurred = cv2.GaussianBlur(img, (5, 5), 0)

        # Convert BGR to HSV
        hsv_img = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Define HSV ranges
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        lower_yellow = np.array([18, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])

        # Masks
        mask_red1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        mask_yellow = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        mask_green = cv2.inRange(hsv_img, lower_green, upper_green)

        # Find contours to detect size
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Thresholds
        min_area = 300  # Minimum area of color blob

        # Analyze detections
        for cnt in contours_red:
            if cv2.contourArea(cnt) > min_area:
                cv2.drawContours(img, [cnt], -1, (0, 0, 255), 3)
                return "RED", img

        for cnt in contours_yellow:
            if cv2.contourArea(cnt) > min_area:
                cv2.drawContours(img, [cnt], -1, (0, 255, 255), 3)
                return "YELLOW", img

        for cnt in contours_green:
            if cv2.contourArea(cnt) > min_area:
                cv2.drawContours(img, [cnt], -1, (0, 255, 0), 3)
                return "GREEN", img

        # If nothing detected
        return None, img

    def timer_callback(self):
        if not self.image_received_flag:
            self.get_logger().info('Waiting for image...')
        self.image_received_flag = False

def main(args=None):
    rclpy.init(args=args)
    cv_e = CVExample()
    rclpy.spin(cv_e)
    cv_e.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
