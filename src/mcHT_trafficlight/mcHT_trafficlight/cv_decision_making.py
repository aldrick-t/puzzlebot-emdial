import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.logging import LoggingSeverity


class CVExample(Node):
    def __init__(self): 
        super().__init__('color_detector')
        self.bridge = CvBridge()

        # declare params (will pull from params.yaml)
        self.declare_parameter('feature_type', 'circle')
        self.declare_parameter('min_area_ratio', 0.1)

        self.feature_type = self.get_parameter('feature_type').value
        self.min_area_ratio = self.get_parameter('min_area_ratio').value

        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.get_logger().info(f"Feature type: {self.feature_type}, min_area_ratio: {self.min_area_ratio}")

        # Sub/pub
        self.sub = self.create_subscription(Image, 'camera/image_raw', self.camera_callback, 10)
        self.pub = self.create_publisher(Image, 'processed_img', 10)
        self.color_pub = self.create_publisher(String, 'traffic_light_color', 10)

        self.image_received_flag = False
        self.create_timer(0.1, self.timer_callback)

        # Single combined debug window
        cv2.namedWindow("Debug View", cv2.WINDOW_NORMAL)

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
        # --- Preprocessing ---
        h, w = img.shape[:2]
        frame_area = h * w
        min_area = frame_area * self.min_area_ratio

        small = cv2.resize(img, (320, 240))
        blur = cv2.GaussianBlur(small, (5,5), 0)
        hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # --- Color masks ---
        lr1, ur1 = np.array([0,100,100]),   np.array([10,255,255])
        lr2, ur2 = np.array([160,100,100]), np.array([180,255,255])
        ly, uy   = np.array([18,100,100]),  np.array([30,255,255])
        lg, ug   = np.array([40,40,40]),    np.array([80,255,255])

        m1 = cv2.inRange(hsv, lr1, ur1)
        m2 = cv2.inRange(hsv, lr2, ur2)
        mask_red    = cv2.bitwise_or(m1, m2)
        mask_yellow = cv2.inRange(hsv, ly, uy)
        mask_green  = cv2.inRange(hsv, lg, ug)

        # --- detect which color first ---
        for color_name, mask, draw_color in [
            ("RED",    mask_red,    (0,0,255)),
            ("YELLOW", mask_yellow, (0,255,255)),
            ("GREEN",  mask_green,  (0,255,0)),
        ]:
            # find contours to see if any large enough
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                self.get_logger().debug(f"{color_name} contour: {area:.1f}")
                if area >= min_area:
                    # we have our color—now isolate the feature!
                    return color_name, self.isolate_feature(small, mask, color_name, draw_color, min_area)

        # if no color, still show debug mosaic
        self.show_mosaic(small, mask_red, mask_yellow, mask_green)
        return None, small

    def isolate_feature(self, img, mask, color_name, draw_color, min_area):
        out = img.copy()
        h, w = mask.shape

        if self.feature_type == 'circle':
            # HoughCircles expects 8-bit gray
            gray = cv2.GaussianBlur(mask, (9,9), 2)
            circles = cv2.HoughCircles(
                gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=h/8,
                param1=50, param2=30,
                minRadius= int(np.sqrt(min_area)/2),
                maxRadius=0
            )
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for (x,y,r) in circles[0,:]:
                    self.get_logger().info(f"{color_name} circle @({x},{y}) r={r}")
                    cv2.circle(out, (x,y), r, draw_color, 2)

        elif self.feature_type == 'square':
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area < min_area: 
                    continue
                # approx polygon
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.04 * peri, True)
                # check for 4 corners + convex
                if len(approx)==4 and cv2.isContourConvex(approx):
                    self.get_logger().info(f"{color_name} square pts={approx.reshape(-1,2).tolist()}")
                    cv2.drawContours(out, [approx], -1, draw_color, 2)

        else:
            self.get_logger().warn(f"Unknown feature_type '{self.feature_type}'")

        # finally show mosaic & return
        self.show_mosaic(img, mask_red=mask if color_name=="RED" else None,
                               mask_yellow=mask if color_name=="YELLOW" else None,
                               mask_green=mask if color_name=="GREEN" else None,
                               processed=out)
        return out

    def show_mosaic(self, processed, mask_red, mask_yellow, mask_green, processed_override=None):
        """Tile: [ swatch | processed | blank ]
                 [ red    | yellow    | green ]"""
        # recreate the HSV swatch
        swatch = np.zeros((180,200,3), np.uint8)
        # ... same swatch‐building as before ...
        # (omitted here for brevity; copy from previous patch)
        sw = cv2.resize(swatch, (320,240))

        proc = processed_override or processed
        proc_resized = cv2.resize(proc, (320,240))
        blank = np.zeros_like(proc_resized)

        def to_bgr(m): return cv2.cvtColor(m, cv2.COLOR_GRAY2BGR) if m is not None else blank

        r = to_bgr(mask_red)
        y = to_bgr(mask_yellow)
        g = to_bgr(mask_green)

        top    = np.hstack([sw, proc_resized, blank])
        bottom = np.hstack([r, y, g])
        mosaic = np.vstack([top, bottom])
        cv2.imshow("Debug View", mosaic)
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