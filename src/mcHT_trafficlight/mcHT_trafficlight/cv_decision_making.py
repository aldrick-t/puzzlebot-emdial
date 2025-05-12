import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import SetParametersResult

class CVExample(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.bridge = CvBridge()

        # --- Declare static parameters (cannot be updated at runtime) ---
        self.declare_parameter('camera_topic', 'camera/image_raw')
        self.declare_parameter('log_severity', 'INFO')

        # --- Declare dynamic parameters (can be updated via rqt_reconfigure) ---
        self.declare_parameter('hsv_ranges.red1_low',  [0, 100, 100])
        self.declare_parameter('hsv_ranges.red1_high', [10, 255, 255])
        self.declare_parameter('hsv_ranges.red2_low',  [160, 100, 100])
        self.declare_parameter('hsv_ranges.red2_high', [180, 255, 255])
        self.declare_parameter('hsv_ranges.yellow_low', [18, 100, 100])
        self.declare_parameter('hsv_ranges.yellow_high',[30, 255, 255])
        self.declare_parameter('hsv_ranges.green_low',  [40,  40,  40])
        self.declare_parameter('hsv_ranges.green_high', [80, 255, 255])
        self.declare_parameter('min_area_ratio', 0.1)

        # Feature and reconfigure not dynamic
        self.declare_parameter('feature_type', 'circle')

        # Parameter callback for dynamic reconfigure
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Initialize parameters
        self.update_parameters()

        # Configure logging severity
        severity = getattr(LoggingSeverity, self.log_severity)
        self.get_logger().set_level(severity)

        # Subscriptions & publishers (topic static)
        self.sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.camera_callback,
            10
        )
        self.pub = self.create_publisher(Image, 'processed_img', 10)
        self.color_pub = self.create_publisher(String, 'traffic_light_color', 10)

        self.image_received_flag = False
        self.create_timer(0.1, self.timer_callback)

        cv2.namedWindow("Debug View", cv2.WINDOW_NORMAL)
        self.get_logger().info('Node initialized with dynamic reconfigure')

    def parameters_callback(self, params):
        # Handle updates to dynamic parameters
        for p in params:
            if p.name.startswith('hsv_ranges.') or p.name == 'min_area_ratio':
                self.get_logger().info(f"Parameter '{p.name}' changed to {p.value}")
            else:
                self.get_logger().warning(f"Attempted to change immutable parameter '{p.name}'")
        # Apply valid updates
        self.update_parameters()
        return SetParametersResult(successful=True)

    def update_parameters(self):
        # Static
        self.camera_topic = self.get_parameter('camera_topic').value
        self.log_severity = self.get_parameter('log_severity').value
        self.feature_type = self.get_parameter('feature_type').value
        # Dynamic HSV masks
        r1_low  = self.get_parameter('hsv_ranges.red1_low').value
        r1_high = self.get_parameter('hsv_ranges.red1_high').value
        r2_low  = self.get_parameter('hsv_ranges.red2_low').value
        r2_high = self.get_parameter('hsv_ranges.red2_high').value
        y_low   = self.get_parameter('hsv_ranges.yellow_low').value
        y_high  = self.get_parameter('hsv_ranges.yellow_high').value
        g_low   = self.get_parameter('hsv_ranges.green_low').value
        g_high  = self.get_parameter('hsv_ranges.green_high').value
        self.hsv_ranges = {
            'red1_low':  np.array(r1_low),  'red1_high':  np.array(r1_high),
            'red2_low':  np.array(r2_low),  'red2_high':  np.array(r2_high),
            'yellow_low':np.array(y_low),  'yellow_high': np.array(y_high),
            'green_low': np.array(g_low),  'green_high':  np.array(g_high)
        }
        self.min_area_ratio = self.get_parameter('min_area_ratio').value

    def camera_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            color, out_img = self.detect_traffic_light_color(cv_img)
            self.pub.publish(self.bridge.cv2_to_imgmsg(out_img, 'bgr8'))
            if color:
                self.color_pub.publish(String(data=color))
                self.get_logger().info(f"Detected color: {color}")
            self.image_received_flag = True
        except Exception as e:
            self.get_logger().error(f"camera_callback error: {e}")

    def detect_traffic_light_color(self, img):
        h, w = img.shape[:2]
        frame_area = h * w
        min_area = frame_area * self.min_area_ratio

        small = cv2.resize(img, (320,240))
        blur  = cv2.GaussianBlur(small, (5,5), 0)
        hsv   = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # Build masks from dynamic HSV ranges
        hr = self.hsv_ranges
        mask_r = cv2.bitwise_or(
            cv2.inRange(hsv, hr['red1_low'], hr['red1_high']),
            cv2.inRange(hsv, hr['red2_low'], hr['red2_high'])
        )
        mask_y = cv2.inRange(hsv, hr['yellow_low'], hr['yellow_high'])
        mask_g = cv2.inRange(hsv, hr['green_low'],  hr['green_high'])

        for color_name, mask in [("RED", mask_r), ("YELLOW", mask_y), ("GREEN", mask_g)]:
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                self.get_logger().debug(f"{color_name} contour area: {area}")
                if area >= min_area:
                    proc = self.isolate_feature(small, mask, color_name, min_area)
                    return color_name, proc

        self.show_mosaic(small, mask_r, mask_y, mask_g)
        return None, small

    def isolate_feature(self, img, mask, color_name, min_area):
        out = img.copy()
        h, w = mask.shape
        draw_color = {
            'RED':    (0,0,255),
            'YELLOW': (0,255,255),
            'GREEN':  (0,255,0)
        }[color_name]

        if self.feature_type == 'circle':
            gray = cv2.GaussianBlur(mask, (9,9), 2)
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=h/8,
                                       param1=50, param2=30,
                                       minRadius=int(np.sqrt(min_area)/2), maxRadius=0)
            if circles is not None:
                for (x,y,r) in np.round(circles[0]).astype(int):
                    self.get_logger().info(f"{color_name} circle @({x},{y}) r={r}")
                    cv2.circle(out,(x,y),r,draw_color,2)

        elif self.feature_type == 'square':
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                if cv2.contourArea(c) < min_area: continue
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.04*peri, True)
                if len(approx)==4 and cv2.isContourConvex(approx):
                    self.get_logger().info(f"{color_name} square pts={approx.reshape(-1,2).tolist()}")
                    cv2.drawContours(out,[approx],-1,draw_color,2)

        self.show_mosaic(img,
                        mask_red=(mask if color_name=='RED' else None),
                        mask_yellow=(mask if color_name=='YELLOW' else None),
                        mask_green=(mask if color_name=='GREEN' else None),
                        processed_override=out)
        return out

    def show_mosaic(self, processed, mask_red, mask_yellow, mask_green, processed_override=None):
        # Build HSV swatch
        swatch = np.zeros((180,200,3), np.uint8)
        def draw_row(low, high, y, label):
            b1 = cv2.cvtColor(np.uint8([[low]]), cv2.COLOR_HSV2BGR)[0,0].tolist()
            b2 = cv2.cvtColor(np.uint8([[high]]),cv2.COLOR_HSV2BGR)[0,0].tolist()
            cv2.rectangle(swatch,(0,y),(100,y+60),b1,-1)
            cv2.rectangle(swatch,(100,y),(200,y+60),b2,-1)
            cv2.putText(swatch,label,(5,y+75),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        hr = self.hsv_ranges
        draw_row(hr['red1_low'], hr['red1_high'], 0, "Red min/max")
        draw_row(hr['yellow_low'],hr['yellow_high'],60,"Yellow min/max")
        draw_row(hr['green_low'],hr['green_high'],120,"Green min/max")
        sw = cv2.resize(swatch,(320,240))

        proc = processed_override or processed
        p = cv2.resize(proc,(320,240))

        # Masks resized
        def to_bgr(m): return cv2.cvtColor(cv2.resize(m,(320,240)),cv2.COLOR_GRAY2BGR) if m is not None else np.zeros((240,320,3),np.uint8)
        mr = to_bgr(mask_red); my = to_bgr(mask_yellow); mg = to_bgr(mask_green)

        # Combined mask overlay
        comb = np.zeros_like(p)
        comb[np.where(mr[:,:,2]>0)] = (0,0,255)
        comb[np.where(my[:,:,1]>0)] = (0,255,255)
        comb[np.where(mg[:,:,1]>0)] = (0,255,0)

        top    = np.hstack([sw, p, comb])
        bottom = np.hstack([mr, my, mg])
        cv2.imshow("Debug View", np.vstack([top,bottom]))
        cv2.waitKey(1)

    def timer_callback(self):
        if not self.image_received_flag:
            self.get_logger().info('Waiting for image...')
        self.image_received_flag = False


def main(args=None):
    rclpy.init(args=args)
    node = CVExample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
