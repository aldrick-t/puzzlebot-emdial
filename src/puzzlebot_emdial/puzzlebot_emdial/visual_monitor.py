import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class VisualMonitor(Node):
    '''
    Visual Monitor Node
    Subscriptions:
    - camera_topic (default: video_source/raw): raw camera image stream
    - gray_processed: grayscale processed image stream
    '''
    def __init__(self):
        super().__init__('visual_monitor')
        self.bridge = CvBridge()

        # Wait for simulated or real ROS time to start
        self.wait_for_ros_time()

        # Set up logging at DEBUG level
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.logger = self.get_logger()
        self.logger.info('VisualMonitor Starting Initialization...')

        # Declare and read parameters
        self.declare_parameter('camera_topic', 'video_source/raw')
        camera_topic = self.get_parameter('camera_topic').value

        # Subscriptions: only two streams
        self.create_subscription(Image, 'tlts_overlay', self.img1_cb, 10)
        self.create_subscription(Image, 'lr_overlay', self.img2_cb, 10)

        # Timer to update display
        self.timer = self.create_timer(0.05, self.timer_cb)
        self.logger.info('VisualMonitor Initialized!')

    def img1_cb(self, msg: Image):
        self.img1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def img2_cb(self, msg: Image):
        self.img2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

    def display_streams(self):
        '''
        Displays two image streams side-by-side in a single window.
        '''
        # Collect available images
        streams = []
        labels = []
        if hasattr(self, 'img1'):
            streams.append(self.img1)
            labels.append('Raw')
        if hasattr(self, 'img2'):
            streams.append(self.img2)
            labels.append('Grayscale')
        
        if not streams:
            self.logger.warning('No image streams available to display')
            return

        # Determine cell size based on max dimensions
        cell_h = max(img.shape[0] for img in streams)
        cell_w = max(img.shape[1] for img in streams)

        # Layout: 1 row x 2 columns
        rows, cols = 1, 2
        canvas_h = cell_h * rows
        canvas_w = cell_w * cols
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

        for idx, img in enumerate(streams):
            # Convert to BGR if grayscale
            if img.ndim == 2 or (img.ndim == 3 and img.shape[2] == 1):
                img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            else:
                img_bgr = img

            # Resize to fit cell
            h, w = img_bgr.shape[:2]
            scale = min(cell_w / w, cell_h / h)
            nw, nh = int(w * scale), int(h * scale)
            resized = cv2.resize(img_bgr, (nw, nh))

            # Center in cell
            cell = np.zeros((cell_h, cell_w, 3), dtype=np.uint8)
            x_off = (cell_w - nw) // 2
            y_off = (cell_h - nh) // 2
            cell[y_off:y_off+nh, x_off:x_off+nw] = resized

            # Label
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = cell_h / 300.0
            thickness = max(1, int(cell_h / 200))
            text = labels[idx]
            (txt_w, txt_h), _ = cv2.getTextSize(text, font, font_scale, thickness)
            txt_x, txt_y = 5, cell_h - 5
            cv2.rectangle(cell, (txt_x-2, txt_y-txt_h-2), (txt_x+txt_w+2, txt_y+2), (20,20,20), -1)
            cv2.putText(cell, text, (txt_x, txt_y), font, font_scale, (255,255,255), thickness, cv2.LINE_AA)

            # Place on canvas
            row = 0
            col = idx
            y0 = row * cell_h
            x0 = col * cell_w
            canvas[y0:y0+cell_h, x0:x0+cell_w] = cell

        # Show in resizable window
        cv2.namedWindow('VisualMonitor', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('VisualMonitor', canvas_w // 2, canvas_h // 2)
        cv2.imshow('VisualMonitor', canvas)
        cv2.waitKey(1)

    def timer_cb(self):
        try:
            self.display_streams()
        except Exception as e:
            self.logger.error(f'Error displaying streams: {e}')

    def wait_for_ros_time(self):
        self.logger.info('Waiting for ROS time to become active...')
        while rclpy.ok():
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.logger.info('ROS time is active!')


def main(args=None):
    rclpy.init(args=args)
    node = VisualMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
