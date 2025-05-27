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
        self.logger = self.get_logger()
        self.logger.info("TLRecogni MK2 Starting Initialization...")
        
        # Static Init default params from config
        # HSV bounds 
        self.declare_parameter('h_r1_min', 0)
        self.declare_parameter('h_r1_max', 20)
        self.declare_parameter('h_r2_min', 154)
        self.declare_parameter('h_r2_max', 179)
        
        self.declare_parameter('h_y_min', 19)
        self.declare_parameter('h_y_max', 45)
        
        self.declare_parameter('h_g_min', 52)
        self.declare_parameter('h_g_max', 97)
        
        self.declare_parameter('s_r1_min', 25)
        self.declare_parameter('s_r1_max', 66)
        self.declare_parameter('s_r2_min', 25)
        self.declare_parameter('s_r2_max', 66)
        
        self.declare_parameter('s_y_min', 17)
        self.declare_parameter('s_y_max', 116)
        
        self.declare_parameter('s_g_min', 40)
        self.declare_parameter('s_g_max', 81)
        
        self.declare_parameter('v_r1_min', 175)
        self.declare_parameter('v_r1_max', 255)
        self.declare_parameter('v_r2_min', 175)
        self.declare_parameter('v_r2_max', 255)
        
        self.declare_parameter('v_y_min', 191)
        self.declare_parameter('v_y_max', 255)
        
        self.declare_parameter('v_g_min', 168)
        self.declare_parameter('v_g_max', 255)
        
        # Dynamic parameters
        self.declare_parameter('camera_topic', 'video_source/raw')
        self.declare_parameter('log_severity', 'INFO')
        self.declare_parameter('min_area', 1)
        self.declare_parameter('max_area', 28)
        self.declare_parameter('min_circularity', 0.5)
        # LAB reference placeholders
        # self.declare_parameter('ref_color_lab_red',    [50,150,150])
        # self.declare_parameter('ref_color_lab_yellow', [80,115,160])
        # self.declare_parameter('ref_color_lab_green',  [70,120,110])
        # Probability threshold for detection (0.0 - 1.0)
        self.declare_parameter('prob_thresh', 0.5)
        
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Utility parameter loads
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.prob_threshold = self.get_parameter('prob_thresh').value
        self.min_circularity = self.get_parameter('min_circularity').value
        
        # HSV bounds parameter loads
        self.lower_red1 = np.array([self.get_parameter('h_r1_min').value,
                                        self.get_parameter('s_r1_min').value,
                                        self.get_parameter('v_r1_min').value])
        self.upper_red1 = np.array([self.get_parameter('h_r1_max').value,
                                        self.get_parameter('s_r1_max').value,
                                        self.get_parameter('v_r1_max').value])
        self.lower_red2 = np.array([self.get_parameter('h_r2_min').value,
                                        self.get_parameter('s_r2_min').value,
                                        self.get_parameter('v_r2_min').value])
        self.upper_red2 = np.array([self.get_parameter('h_r2_max').value,
                                        self.get_parameter('s_r2_max').value,
                                        self.get_parameter('v_r2_max').value])
        self.lower_yellow = np.array([self.get_parameter('h_y_min').value,
                                        self.get_parameter('s_y_min').value,
                                        self.get_parameter('v_y_min').value])
        self.upper_yellow = np.array([self.get_parameter('h_y_max').value,
                                        self.get_parameter('s_y_max').value,
                                        self.get_parameter('v_y_max').value])
        self.lower_green = np.array([self.get_parameter('h_g_min').value,
                                        self.get_parameter('s_g_min').value,
                                        self.get_parameter('v_g_min').value])
        self.upper_green = np.array([self.get_parameter('h_g_max').value,
                                        self.get_parameter('s_g_max').value,
                                        self.get_parameter('v_g_max').value])
        
        # # Initial LAB refs
        # self.ref_colors_lab = {
        #     'red':    self.get_parameter('ref_color_lab_red').value,
        #     'yellow': self.get_parameter('ref_color_lab_yellow').value,
        #     'green':  self.get_parameter('ref_color_lab_green').value,
        # }

        
        # Create tuners for HSV and LAB
        #self._make_color_tuners()
        #self._make_lab_tuners()
        # Timer to spin OpenCV GUI
        self.create_timer(1/30.0, self._spin_gui)
        
        # Subscriptions
        self.create_subscription(Image,self.get_parameter('camera_topic').value,self.camera_callback,10)
        
        # Publishers
        self.tlight_status_pub = self.create_publisher(String, 'traffic_light_color', 10)
        self.tlight_overlay_pub = self.create_publisher(Image,  'tlight_overlay', 10)
        self.hsvmask_pub = self.create_publisher(Image, 'hsv_mask', 10)
        self.contour_pub = self.create_publisher(Image, 'contour_overlay', 10)
        
        self.logger.info("TLRecogni MK2 Initialization Complete")

    def _make_color_tuners(self):
        """Create HSV trackbar windows for Red1, Red2, Yellow, and Green."""
        def nothing(x): pass
        for color in ('Red1','Red2','Yellow','Green'):
            win = f"{color} Tuner"
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
            # get default lower/upper arrays
            lower = getattr(self, f'lower_{color.lower()}')
            upper = getattr(self, f'upper_{color.lower()}')
            # H ranges
            cv2.createTrackbar('Hmin', win, int(lower[0]), 179, nothing)
            cv2.createTrackbar('Hmax', win, int(upper[0]), 179, nothing)
            # S ranges
            cv2.createTrackbar('Smin', win, int(lower[1]), 255, nothing)
            cv2.createTrackbar('Smax', win, int(upper[1]), 255, nothing)
            # V ranges
            cv2.createTrackbar('Vmin', win, int(lower[2]), 255, nothing)
            cv2.createTrackbar('Vmax', win, int(upper[2]), 255, nothing)

    def _read_color_tuner(self, color):
        """Read HSV bounds from a given color tuner window."""
        win = f"{color} Tuner"
        h1 = cv2.getTrackbarPos('Hmin', win)
        s1 = cv2.getTrackbarPos('Smin', win)
        v1 = cv2.getTrackbarPos('Vmin', win)
        h2 = cv2.getTrackbarPos('Hmax', win)
        s2 = cv2.getTrackbarPos('Smax', win)
        v2 = cv2.getTrackbarPos('Vmax', win)
        return np.array([h1,s1,v1]), np.array([h2,s2,v2])

    # def _make_lab_tuners(self):
    #     """Create LAB reference trackbars for Red, Yellow, and Green."""
    #     def nothing(x): pass
    #     for color in ('Red','Yellow','Green'):
    #         win = f"{color} Lab Tuner"
    #         init = self.ref_colors_lab[color.lower()]
    #         cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    #         cv2.createTrackbar('L', win, init[0], 255, nothing)
    #         cv2.createTrackbar('a', win, init[1], 255, nothing)
    #         cv2.createTrackbar('b', win, init[2], 255, nothing)

    # def _read_lab_tuner(self, color):
    #     """Read LAB reference values from a given lab tuner window."""
    #     win = f"{color} Lab Tuner"
    #     l = cv2.getTrackbarPos('L', win)
    #     a = cv2.getTrackbarPos('a', win)
    #     b = cv2.getTrackbarPos('b', win)
    #     return [l,a,b]

    def _spin_gui(self):
        """Pump the OpenCV GUI event loop."""
        cv2.waitKey(1)

    def camera_callback(self, msg: Image):
        # Convert ROS Image to OpenCV BGR
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Preprocess: blur + CLAHE
        proc_image = self.preprocess(cv_image)
        # to HSV
        self.hsv_image = cv2.cvtColor(proc_image, cv2.COLOR_BGR2HSV)

        # Draw threshold lines based on probability threshold
        h, w = cv_image.shape[:2]
        cx = w / 2.0
        left_x  = int(cx * (1 - self.prob_threshold))
        right_x = int(cx * (1 + self.prob_threshold))
        cv2.line(cv_image, (left_x,0), (left_x,h), (255,0,0), 2)
        cv2.line(cv_image, (right_x,0),(right_x,h), (255,0,0), 2)

        # Read and apply HSV tuners
        # self.lower_red1,   self.upper_red1   = self._read_color_tuner('Red1')
        # self.lower_red2,   self.upper_red2   = self._read_color_tuner('Red2')
        # self.lower_yellow, self.upper_yellow = self._read_color_tuner('Yellow')
        # self.lower_green,  self.upper_green  = self._read_color_tuner('Green')

        red_mask1   = cv2.inRange(self.hsv_image, self.lower_red1,   self.upper_red1)
        red_mask2   = cv2.inRange(self.hsv_image, self.lower_red2,   self.upper_red2)
        yellow_mask = cv2.inRange(self.hsv_image, self.lower_yellow, self.upper_yellow)
        green_mask  = cv2.inRange(self.hsv_image, self.lower_green,  self.upper_green)
        # cv2.imshow('Red1 Tuner',   red_mask1)
        # cv2.imshow('Red2 Tuner',   red_mask2)
        # cv2.imshow('Yellow Tuner', yellow_mask)
        # cv2.imshow('Green Tuner',  green_mask)

        mask = cv2.bitwise_or(red_mask1, red_mask2)
        mask = cv2.bitwise_or(mask, yellow_mask)
        mask = cv2.bitwise_or(mask, green_mask)
        mask = self.clean_mask(mask)

        # # Update LAB references
        # for color in ('Red','Yellow','Green'):
        #     self.ref_colors_lab[color.lower()] = self._read_lab_tuner(color)

        # Detect circles and get overlay of contours
        circles, contour_overlay = self.find_color_circles(mask, self.min_area, self.min_circularity)

        # Classify, filter by probability, annotate centroids
        labels = []
        for (x,y), r, area, circ in circles:
            prob = abs(x - cx) / cx
            if prob < self.prob_threshold:
                continue
            # mark centroid
            cv2.circle(cv_image, (int(x),int(y)), 3, (0,0,255), -1)
            cv2.putText(cv_image, f"({int(x)},{int(y)})",
                        (int(x+5), int(y-5)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255,255,0), 1)
            # classify color
            #label = self.classify_circle_lab(cv_image, (int(x),int(y)), r, self.ref_colors_lab)
            label = self.classify_circle_hsv(cv_image, (int(x),int(y)), r,
                                             red_mask1, red_mask2,
                                             yellow_mask, green_mask)
            labels.append(label)
            # draw detected circle
            cv2.circle(cv_image, (int(x),int(y)), int(r), (0,255,0), 2)
            cv2.putText(cv_image, f"{label}:{prob:.2f}",
                        (int(x-10), int(y-10)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255,255,255), 2)

        # Publish detected color(s)
        color_msg = String()
        color_msg.data = ','.join(labels) if labels else 'none'
        self.tlight_status_pub.publish(color_msg)

        # Publish overlay image
        overlay_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        overlay_msg.header = msg.header
        self.tlight_overlay_pub.publish(overlay_msg)

        # Publish mask and contour overlay
        mask_msg    = self.bridge.cv2_to_imgmsg(mask,           encoding='mono8')
        contour_msg = self.bridge.cv2_to_imgmsg(contour_overlay, encoding='mono8')
        mask_msg.header    = msg.header
        contour_msg.header = msg.header
        self.hsvmask_pub.publish(mask_msg)
        self.contour_pub.publish(contour_msg)

    def preprocess(self, img):
        """Blur → LAB → CLAHE on L → back to BGR"""
        blur = cv2.GaussianBlur(img, (7,7), 0)
        lab  = cv2.cvtColor(blur, cv2.COLOR_BGR2LAB)
        l,a,b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        l_eq  = clahe.apply(l)
        lab_eq= cv2.merge((l_eq, a, b))
        return cv2.cvtColor(lab_eq, cv2.COLOR_LAB2BGR)

    def clean_mask(self, mask):
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        m = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
        m = cv2.morphologyEx(m,    cv2.MORPH_CLOSE, kernel, iterations=1)
        return m

    def find_color_circles(self, mask, min_area, min_circ):
        out = []
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            peri = cv2.arcLength(c, True)
            circ = 4*np.pi*area/(peri*peri) if peri>0 else 0
            if circ < min_circ:
                continue
            (x,y), r = cv2.minEnclosingCircle(c)
            out.append(((x,y), r, area, circ))
            cv2.putText(mask, f"{int(area)}", (int(x)-10, int(y)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        return out, mask

    # def classify_circle_lab(self, bgr_img, center, radius, ref_colors_lab):
    #     mask = np.zeros(bgr_img.shape[:2], dtype=np.uint8)
    #     cv2.circle(mask, center, int(radius), 255, -1)
    #     lab = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2LAB)
    #     meanL, meana, meanb, _ = cv2.mean(lab, mask=mask)
    #     best_label, best_dist = None, float('inf')
    #     for label, rgb in ref_colors_lab.items():
    #         rL, ra, rb = rgb
    #         d = (meanL-rL)**2 + (meana-ra)**2 + (meanb-rb)**2
    #         if d < best_dist:
    #             best_dist, best_label = d, label
    #     return best_label
    def classify_circle_hsv(self, bgr_img, center, radius, redmask1, redmask2, yellowmask, greenmask):
        mask = np.zeros(bgr_img.shape[:2], dtype=np.uint8)
        cv2.circle(mask, center, int(radius), 255, -1)
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        best_label = 'none'
        if redmask1[int(center[1]), int(center[0])] > 0:
            best_label = 'red'
        elif redmask2[int(center[1]), int(center[0])] > 0:
            best_label = 'red'
        elif yellowmask[int(center[1]), int(center[0])] > 0:
            best_label = 'yellow'
        elif greenmask[int(center[1]), int(center[0])] > 0:
            best_label = 'green'
        return best_label

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'log_severity':
                lvl = param.value
                if lvl in ('DEBUG','INFO','WARN','ERROR'):
                    self.get_logger().set_level(getattr(LoggingSeverity, lvl))
                else:
                    self.get_logger().error('Invalid log severity level')
            elif param.name == 'camera_topic':
                self.get_logger().info(f"Camera topic changed to {param.value}")
            elif param.name == 'min_area':
                self.min_area = param.value
                self.get_logger().info(f"Min area changed to {self.min_area}")
            elif param.name == 'min_circularity':
                self.min_circularity = param.value
                self.get_logger().info(f"Min circularity changed to {self.min_circularity}")
            elif param.name == 'max_area':
                self.max_area = param.value
                self.get_logger().info(f"Max area changed to {self.max_area}")
            elif param.name == 'prob_thresh':
                self.prob_threshold = param.value
                self.get_logger().info(f"Probability threshold set to {param.value}")
        
        
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
