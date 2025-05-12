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

        # DEBUG logging 
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.get_logger().debug('Initializing ColorDetectorNode!')

        # Subscribers & publishers
        # Subscribe to the camera image topic
        # Change Camera topic to match setup
        # Gazebo: /camera, Real Remote: /image_source/raw, Real Local: /camera/image_raw
        self.sub = self.create_subscription(Image, 'camera/image_raw', self.camera_callback, 10)
        self.pub = self.create_publisher(Image, 'processed_img', 10)
        self.color_pub = self.create_publisher(String, 'traffic_light_color', 10)

        self.image_received_flag = False
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('ros_color_tracker Node started')

        # Single combined debug window
        cv2.namedWindow("Debug View", cv2.WINDOW_NORMAL)

    def camera_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            color_detected, processed_img = self.detect_traffic_light_color(cv_img)

            # Publish processed image
            self.pub.publish(self.bridge.cv2_to_imgmsg(processed_img, 'bgr8'))

            # Publish detected color
            if color_detected:
                self.color_pub.publish(String(data=color_detected))
                self.get_logger().info(f'Detected color: {color_detected}')

            self.image_received_flag = True
        except Exception as e:
            self.get_logger().info(f'Failed to process image: {str(e)}')

    def detect_traffic_light_color(self, img):
        # Resize & blur
        img = cv2.resize(img, (320, 240))
        blurred = cv2.GaussianBlur(img, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # HSV ranges
        lr1, ur1 = np.array([0,100,100]),   np.array([10,255,255])
        lr2, ur2 = np.array([160,100,100]), np.array([180,255,255])
        ly, uy   = np.array([18,100,100]),  np.array([30,255,255])
        lg, ug   = np.array([40,40,40]),    np.array([80,255,255])

        # Masks
        mr1 = cv2.inRange(hsv, lr1, ur1)
        mr2 = cv2.inRange(hsv, lr2, ur2)
        mask_red    = cv2.bitwise_or(mr1, mr2)
        mask_yellow = cv2.inRange(hsv, ly, uy)
        mask_green  = cv2.inRange(hsv, lg, ug)

        # Build HSV swatch (180×200) then resize to 240×320
        swatch = np.zeros((180,200,3), np.uint8)
        def draw_row(min_hsv, max_hsv, y):
            bmin = cv2.cvtColor(np.uint8([[min_hsv]]), cv2.COLOR_HSV2BGR)[0,0].tolist()
            bmax = cv2.cvtColor(np.uint8([[max_hsv]]), cv2.COLOR_HSV2BGR)[0,0].tolist()
            cv2.rectangle(swatch, (0,y),(100,y+60), bmin, -1)
            cv2.rectangle(swatch, (100,y),(200,y+60), bmax, -1)
        draw_row(lr1, ur1, 0);    cv2.putText(swatch,"Red min/max",(5,75),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        draw_row(ly,  uy,   60);   cv2.putText(swatch,"Yellow min/max",(5,135),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        draw_row(lg,  ug,   120);  cv2.putText(swatch,"Green min/max",(5,175),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        sw_resized = cv2.resize(swatch, (320, 240))

        # Convert masks to BGR
        mr_bgr = cv2.cvtColor(mask_red,    cv2.COLOR_GRAY2BGR)
        my_bgr = cv2.cvtColor(mask_yellow, cv2.COLOR_GRAY2BGR)
        mg_bgr = cv2.cvtColor(mask_green,  cv2.COLOR_GRAY2BGR)

        # Find contours & log
        cnts_r, _ = cv2.findContours(mask_red,    cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts_y, _ = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts_g, _ = cv2.findContours(mask_green,  cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        min_area = 300
        detected = None

        # helper to check contours
        def check(cnts, color_name, draw_color):
            nonlocal detected
            for c in cnts:
                area = cv2.contourArea(c)
                self.get_logger().debug(f"{color_name} contour area: {area}")
                if area > min_area and detected is None:
                    cv2.drawContours(img, [c], -1, draw_color, 3)
                    detected = color_name

        check(cnts_r, "RED",    (0,0,255))
        check(cnts_y, "YELLOW", (0,255,255))
        check(cnts_g, "GREEN",  (0,255,0))

        # Create blank slot for 6th cell
        blank = np.zeros_like(img)

        # Top row: swatch | processed | blank
        top = np.hstack([sw_resized, img, blank])
        # Bottom row: red mask | yellow mask | green mask
        bottom = np.hstack([mr_bgr, my_bgr, mg_bgr])

        # Full mosaic
        mosaic = np.vstack([top, bottom])
        cv2.imshow("Debug View", mosaic)
        cv2.waitKey(1)

        return detected, img

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