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

        # Static parameters (immutable at runtime)
        self.declare_parameter('camera_topic', 'camera/image_raw')
        self.declare_parameter('log_severity', 'INFO')
        self.declare_parameter('feature_type', 'circle')

        # Dynamic parameters (updatable via rqt_reconfigure)
        self.declare_parameter('red1_low',  [0, 150, 150])
        self.declare_parameter('red1_high', [5, 255, 255])
        self.declare_parameter('red2_low',  [175, 100, 100])
        self.declare_parameter('red2_high', [180, 255, 255])
        self.declare_parameter('yellow_low',  [18, 100, 100])
        self.declare_parameter('yellow_high', [30, 255, 255])
        self.declare_parameter('green_low',   [40,  40,  40])
        self.declare_parameter('green_high',  [80, 255, 255])
        self.declare_parameter('min_area_ratio', 0.01)

        # Register parameter-change callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.update_parameters()

        # Apply log level
        level = getattr(LoggingSeverity, self.log_severity, LoggingSeverity.INFO)
        self.get_logger().set_level(level)
        self.get_logger().info(f"Initialized with topic={self.camera_topic}, feature={self.feature_type}, min_area_ratio={self.min_area_ratio}")

        # Subscriber & publishers
        self.sub = self.create_subscription(Image, self.camera_topic, self.camera_callback, 10)
        self.pub = self.create_publisher(Image, 'processed_img', 10)
        self.color_pub = self.create_publisher(String, 'traffic_light_color', 10)

        self.image_received_flag = False
        self.create_timer(0.1, self.timer_callback)

        cv2.namedWindow('Debug View', cv2.WINDOW_NORMAL)

    def parameters_callback(self, params):
        for p in params:
            if p.name in ('red1_low','red1_high','red2_low','red2_high',
                          'yellow_low','yellow_high','green_low','green_high',
                          'min_area_ratio'):
                self.get_logger().info(f"Param '{p.name}' changed to {p.value}")
            else:
                self.get_logger().warning(f"Immutable param '{p.name}' cannot be updated at runtime")
        self.update_parameters()
        return SetParametersResult(successful=True)

    def update_parameters(self):
        # Static
        self.camera_topic = self.get_parameter('camera_topic').value
        self.log_severity = self.get_parameter('log_severity').value
        self.feature_type = self.get_parameter('feature_type').value

        # Dynamic HSV
        self.hsv_ranges = {
            'red1_low':  np.array(self.get_parameter('red1_low').value),
            'red1_high': np.array(self.get_parameter('red1_high').value),
            'red2_low':  np.array(self.get_parameter('red2_low').value),
            'red2_high': np.array(self.get_parameter('red2_high').value),
            'yellow_low':  np.array(self.get_parameter('yellow_low').value),
            'yellow_high': np.array(self.get_parameter('yellow_high').value),
            'green_low':   np.array(self.get_parameter('green_low').value),
            'green_high':  np.array(self.get_parameter('green_high').value)
        }
        self.min_area_ratio = self.get_parameter('min_area_ratio').value

    def camera_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            color, out_img = self.detect_color_and_feature(cv_img)

            # Publish processed image
            self.pub.publish(self.bridge.cv2_to_imgmsg(out_img, 'bgr8'))

            # Publish detected color
            if color is not None:
                self.color_pub.publish(String(data=color))
                self.get_logger().info(f"Detected color: {color}")

            self.image_received_flag = True
        except Exception as e:
            self.get_logger().error(f"camera_callback error: {e}")

    def detect_color_and_feature(self, img):
        # Resize early for threshold consistency
        small = cv2.resize(img, (320, 240))
        h, w = small.shape[:2]
        frame_area = h * w
        min_area = frame_area * self.min_area_ratio

        blur = cv2.GaussianBlur(small, (3, 3), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        
        hr = self.hsv_ranges
        mask_r = cv2.bitwise_or(
            cv2.inRange(hsv, hr['red1_low'], hr['red1_high']),
            cv2.inRange(hsv, hr['red2_low'], hr['red2_high'])
        )
        mask_y = cv2.inRange(hsv, hr['yellow_low'], hr['yellow_high'])
        mask_g = cv2.inRange(hsv, hr['green_low'],  hr['green_high'])

        # Apply morphological operations to reduce noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        # mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_CLOSE, kernel)
        mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN, kernel)
        # mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_CLOSE, kernel)
        mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN, kernel)
        # mask_g = cv2.morphologyEx(mask_g, cv2.MORPH_CLOSE, kernel)
        mask_g = cv2.morphologyEx(mask_g, cv2.MORPH_OPEN, kernel)

        # reset last feature
        self.last_feature = None

        # detect color via contours
        for name, mask in [('RED',mask_r),('YELLOW',mask_y),('GREEN',mask_g)]:
            cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                self.get_logger().debug(f"{name} contour area: {area}")
                if area < min_area:
                    continue
                # found a valid color region -> isolate shape
                out = self.isolate_shape(small, mask, name, c, min_area)
                return name, out

        # no detection
        self.show_mosaic(small, mask_r, mask_y, mask_g)
        return None, small

    def isolate_shape(self, img, mask, color_name, contour, min_area):
        out = img.copy()
        draw_color = {'RED': (0, 0, 255), 'YELLOW': (0, 255, 255), 'GREEN': (0, 255, 0)}[color_name]

        if self.feature_type == 'circle':
            # Use minimum enclosing circle
            (x, y), r = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(r)
            circle_area = np.pi * (radius ** 2)
            if circle_area < min_area:
                self.get_logger().info(f"{color_name} circle rejected (area={circle_area:.1f} < min_area={min_area:.1f})")
                return img
            self.get_logger().info(f"{color_name} circle @({center[0]},{center[1]}) r={radius} area={circle_area:.1f}")
            cv2.circle(out, center, radius, draw_color, 2)
            self.last_feature = ('circle', (center, radius))

        else:  # square
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                area = cv2.contourArea(approx)
                if area < min_area:
                    self.get_logger().info(f"{color_name} square rejected (area={area:.1f} < min_area={min_area:.1f})")
                    return img
                self.get_logger().info(f"{color_name} square corners={approx.reshape(-1, 2).tolist()}")
                cv2.drawContours(out, [approx], -1, draw_color, 2)
                self.last_feature = ('square', approx)

        self.show_mosaic(img, mask_r=(mask if color_name == 'RED' else None),
                               mask_y=(mask if color_name == 'YELLOW' else None),
                               mask_g=(mask if color_name == 'GREEN' else None),
                               processed_override=out)
        return out

    def show_mosaic(self, processed, mask_r, mask_y, mask_g, processed_override=None):
        # Build HSV swatch
        swatch = np.zeros((180,200,3), np.uint8)
        def draw_row(low, high, y, label):
            c1 = cv2.cvtColor(np.uint8([[low]]), cv2.COLOR_HSV2BGR)[0,0].tolist()
            c2 = cv2.cvtColor(np.uint8([[high]]),cv2.COLOR_HSV2BGR)[0,0].tolist()
            cv2.rectangle(swatch,(0,y),(100,y+60),c1,-1)
            cv2.rectangle(swatch,(100,y),(200,y+60),c2,-1)
            cv2.putText(swatch,label,(5,y+75),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        hr = self.hsv_ranges
        draw_row(hr['red1_low'],  hr['red1_high'],  0,  'Red min/max')
        draw_row(hr['yellow_low'],hr['yellow_high'],60, 'Yellow min/max')
        draw_row(hr['green_low'], hr['green_high'], 120,'Green min/max')
        sw = cv2.resize(swatch, (320,240))

        proc_img = processed_override if processed_override is not None else processed
        proc = cv2.resize(proc_img, (320,240))

        # Masks to BGR
        def to_bgr(m): return cv2.cvtColor(cv2.resize(m,(320,240)),cv2.COLOR_GRAY2BGR)
        mr = to_bgr(mask_r) if mask_r is not None else np.zeros((240,320,3),np.uint8)
        my = to_bgr(mask_y) if mask_y is not None else np.zeros((240,320,3),np.uint8)
        mg = to_bgr(mask_g) if mask_g is not None else np.zeros((240,320,3),np.uint8)

        # Combined mask
        comb = np.zeros_like(proc)
        comb[np.where(mr[:,:,2]>0)] = (0,0,255)
        comb[np.where(my[:,:,1]>0)] = (0,255,255)
        comb[np.where(mg[:,:,1]>0)] = (0,255,0)

        # Blue overlay on detected feature
        if self.last_feature is not None:
            ftype, val = self.last_feature
            if ftype == 'circle':
                (cx,cy), cr = val
                cv2.circle(comb, (cx,cy), cr, (255,0,0), 2)
            else:
                cv2.drawContours(comb, [val], -1, (255,0,0), 2)

        top    = np.hstack([sw, proc, comb])
        bottom = np.hstack([mr, my, mg])
        cv2.imshow('Debug View', np.vstack([top,bottom]))
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
