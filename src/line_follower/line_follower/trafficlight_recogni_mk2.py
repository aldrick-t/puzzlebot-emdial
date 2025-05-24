'''
trafficlight_recogni_mk2.py

Traffic Light Recognition Node
Improved Traffic Light Detection with Advanced OpenCV Methods

aldrick-t
MAY 2025
'''

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import SetParametersResult

class TLightRecogniMK2(Node):
    def __init__(self):
        super().__init__('trafficlight_recogni_mk2')
        self.bridge = CvBridge()
        
        self.wait_for_ros_time()
        
        # Logger initialization
        # Set logging level default to DEBUG for all nodes
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        rclpy.logging.get_logger('rclpy').set_level(LoggingSeverity.DEBUG)
        
        # Start logger
        self.logger = self.get_logger()
        
        # Init message
        self.logger.info("TLRecogni MK2 Starting Initialization...")
        
        # Dynamic parameters
        self.declare_parameter('camera_topic', 'video_source/raw')
        self.declare_parameter('log_severity', 'INFO')
        
        self.declare_parameter('min_area', 100)
        self.declare_parameter('max_area', 10000)
        
        self.declare_parameter('min_radius', 10)
        self.declare_parameter('max_radius', 100)
        
        self.declare_parameter('min_circularity', 0.5)
        self.declare_parameter('min_convexity', 0.5)
        self.declare_parameter('min_inertia_ratio', 0.5)
        
        self.declare_parameter('color_threshold', 50)
        
        # HSV params (initial placeholders, will be overridden by trackbars)
        self.declare_parameter('hsv_lower',  [0,  30,  30])
        self.declare_parameter('hsv_upper', [179, 255, 255])
        
        # LAB reference colors
        self.declare_parameter('ref_color_lab_red',    [50, 150, 150])
        self.declare_parameter('ref_color_lab_yellow', [80, 115, 160])
        self.declare_parameter('ref_color_lab_green',  [70, 120, 110])
        
        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Load static parameters into members
        self.min_area = self.get_parameter('min_area').value
        self.min_radius = self.get_parameter('min_radius').value
        self.min_circularity = self.get_parameter('min_circularity').value
        self.max_area = self.get_parameter('max_area').value
        self.max_radius = self.get_parameter('max_radius').value
        
        self.hsv_low  = np.array(self.get_parameter('hsv_lower').value)
        self.hsv_high = np.array(self.get_parameter('hsv_upper').value)
        
        self.ref_colors_lab = {
            'red':    self.get_parameter('ref_color_lab_red').value,
            'yellow': self.get_parameter('ref_color_lab_yellow').value,
            'green':  self.get_parameter('ref_color_lab_green').value,
        }
        
        # Create tuning windows for live HSV calibration
        self._make_color_tuners()
        # Timer to pump OpenCV GUI loop at ~30Hz
        self.create_timer(1/30.0, self._spin_gui)
        
        # Subscriptions
        self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.camera_callback,
            10)
        
        # Publishers
        self.tlight_status_pub  = self.create_publisher(String, 'traffic_light_color', 10)
        self.tlight_overlay_pub = self.create_publisher(Image,  'tlight_overlay',   10)
        self.hsvmask_pub        = self.create_publisher(Image,  'hsv_mask',         10)
        self.contour_pub        = self.create_publisher(Image,  'contour_overlay',  10)
        
        self.logger.info("TLRecogni MK2 Initialization Complete")

    def _make_color_tuners(self):
        """Create OpenCV windows and trackbars for Red, Yellow, Green HSV tuning."""
        def nothing(x): pass
        for color in ('Red','Yellow','Green'):
            win = f"{color} Tuner"
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
            cv2.createTrackbar('Hmin', win, 0,   179, nothing)
            cv2.createTrackbar('Hmax', win, 179, 179, nothing)
            cv2.createTrackbar('Smin', win, 0,   255, nothing)
            cv2.createTrackbar('Smax', win, 255, 255, nothing)
            cv2.createTrackbar('Vmin', win, 0,   255, nothing)
            cv2.createTrackbar('Vmax', win, 255, 255, nothing)

    def _read_color_tuner(self, color):
        """Read trackbar positions for a given color tuner window."""
        win = f"{color} Tuner"
        h1 = cv2.getTrackbarPos('Hmin', win)
        s1 = cv2.getTrackbarPos('Smin', win)
        v1 = cv2.getTrackbarPos('Vmin', win)
        h2 = cv2.getTrackbarPos('Hmax', win)
        s2 = cv2.getTrackbarPos('Smax', win)
        v2 = cv2.getTrackbarPos('Vmax', win)
        return np.array([h1,s1,v1]), np.array([h2,s2,v2])

    def _spin_gui(self):
        """Pump the OpenCV GUI event loop."""
        cv2.waitKey(1)

    def camera_callback(self, msg: Image):
        # Convert ROS Image to OpenCV BGR
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Preprocess: blur + CLAHE in LAB
        proc_image = self.preprocess(cv_image)
        # HSV conversion
        self.hsv_image = cv2.cvtColor(proc_image, cv2.COLOR_BGR2HSV)

        # Live-read and apply trackbar-tuned HSV ranges
        self.lower_red, self.upper_red       = self._read_color_tuner('Red')
        self.lower_yellow, self.upper_yellow = self._read_color_tuner('Yellow')
        self.lower_green, self.upper_green   = self._read_color_tuner('Green')

        # Generate and show individual masks
        red_mask    = cv2.inRange(self.hsv_image, self.lower_red,   self.upper_red)
        yellow_mask = cv2.inRange(self.hsv_image, self.lower_yellow,self.upper_yellow)
        green_mask  = cv2.inRange(self.hsv_image, self.lower_green, self.upper_green)
        cv2.imshow('Red Tuner',    red_mask)
        cv2.imshow('Yellow Tuner', yellow_mask)
        cv2.imshow('Green Tuner',  green_mask)

        # Combine into one mask, then clean
        mask = cv2.bitwise_or(red_mask, yellow_mask)
        mask = cv2.bitwise_or(mask, green_mask)
        mask = self.clean_mask(mask)

        # Find circles + contour overlay
        circles, contour_overlay = self.find_color_circles(
            mask, self.min_area, self.max_area, self.min_circularity)

        # Classify and annotate
        labels = []
        for (x,y), r, area, circ in circles:
            label = self.classify_circle_lab(cv_image, (x,y), r, self.ref_colors_lab)
            labels.append(label)
            cv2.circle(cv_image, (x,y), r, (0,255,0), 1)
            cv2.putText(cv_image, label, (x-10,y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)

        # Publish detected color string
        color_msg = String()
        color_msg.data = ','.join(labels) if labels else 'none'
        self.tlight_status_pub.publish(color_msg)

        # Publish annotated overlay image
        overlay_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        overlay_msg.header = msg.header
        self.tlight_overlay_pub.publish(overlay_msg)

        # Publish mask and contour overlay as mono8
        mask_msg    = self.bridge.cv2_to_imgmsg(mask,           encoding='mono8')
        contour_msg = self.bridge.cv2_to_imgmsg(contour_overlay, encoding='mono8')
        mask_msg.header    = msg.header
        contour_msg.header = msg.header
        self.hsvmask_pub.publish(mask_msg)
        self.contour_pub.publish(contour_msg)

    # ... existing tl_detect_sys left unchanged ...
    def tl_detect_sys(self, image):
        # Unused helper from earlier versions
        return

    def preprocess(self, img):
        """Blur → LAB → CLAHE on L → back to BGR"""
        blur = cv2.GaussianBlur(img, (7,7), 0)
        lab  = cv2.cvtColor(blur, cv2.COLOR_BGR2LAB)
        l,a,b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        l_eq  = clahe.apply(l)
        lab_eq = cv2.merge((l_eq, a, b))
        return cv2.cvtColor(lab_eq, cv2.COLOR_LAB2BGR)

    def clean_mask(self, mask):
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        m = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
        m = cv2.morphologyEx(m,    cv2.MORPH_CLOSE, kernel, iterations=1)
        return m

    def find_color_circles(self, mask, min_area, max_area, min_circ):
        out = []
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_area or area > max_area:
                continue
            peri = cv2.arcLength(c, True)
            circ = 4*np.pi*area/(peri*peri) if peri > 0 else 0
            if circ < min_circ:
                continue
            (x, y), r = cv2.minEnclosingCircle(c)
            out.append(((int(x), int(y)), int(r), area, circ))
            cv2.putText(mask, f"{int(area)}", (int(x)-10, int(y)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        return out, mask

    def classify_circle_lab(self, bgr_img, center, radius, ref_colors_lab):
        mask = np.zeros(bgr_img.shape[:2], dtype=np.uint8)
        cv2.circle(mask, center, radius, 255, -1)
        lab = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2LAB)
        meanL, meana, meanb, _ = cv2.mean(lab, mask=mask)
        best_label, best_dist = None, float('inf')
        for label, rgb in ref_colors_lab.items():
            rL, ra, rb = rgb
            d = (meanL-rL)**2 + (meana-ra)**2 + (meanb-rb)**2
            if d < best_dist:
                best_dist, best_label = d, label
        return best_label

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'log_severity':
                lvl = param.value
                if lvl in ('DEBUG','INFO','WARN','ERROR'):
                    sev = getattr(LoggingSeverity, lvl)
                    self.get_logger().set_level(sev)
                else:
                    self.get_logger().error('Invalid log severity level')
            elif param.name == 'camera_topic':
                self.get_logger().info(f"Camera topic changed to {param.value}")
            elif param.name == 'min_area':
                self.min_area = param.value
                self.get_logger().info(f"Min area changed to {self.min_area}")
            elif param.name == 'min_radius':
                self.min_radius = param.value
                self.get_logger().info(f"Min radius changed to {self.min_radius}")
            elif param.name == 'min_circularity':
                self.min_circularity = param.value
                self.get_logger().info(f"Min circularity changed to {self.min_circularity}")
            elif param.name == 'max_area':
                self.max_area = param.value
                self.get_logger().info(f"Max area changed to {self.max_area}")
            elif param.name == 'max_radius':
                self.max_radius = param.value
                self.get_logger().info(f"Max radius changed to {self.max_radius}")
            
            
        return SetParametersResult(successful=True)

    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('ROS time is active!')


def main(args=None):
    rclpy.init(args=args)
    node = TLightRecogniMK2()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
