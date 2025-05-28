''' 
visual_monitor.py

Visual Monitor Node
Node subscribes to multple camera stream topics and displays them in a window.
Node for visualization and debugging.
Requires GUI support (display and window management).
Uses OpenCV for image management.

aldrick-t
MAY 2025
'''

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import SetParametersResult

class VisualMonitor(Node):
    '''
    Visual Monitor Node
    Subscriptions:
    - /"undefined raw camera stream": Camera stream topic
    Publishers:
    - None
    '''
    def __init__(self):
        super().__init__('visual_monitor')
        self.bridge = CvBridge()
        
        self.wait_for_ros_time()
        
        # Logger initialization 
        # Set logging level default to DEBUG for all nodes
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG)
        # Start logger
        self.logger = self.get_logger()
        
        #Init message
        self.logger.info("VisualMonitor Starting Initialization...")
        
        #Parameters immutable after startup sequence
        # Video source (determined by launch mode)
        self.declare_parameter('camera_topic', 'video_source/raw')
        
        # Parameter Callback
        # self.add_on_set_parameters_callback(self.parameter_callback)
    
        # Subscriptions
        # Camera stream topic
        self.create_subscription(Image, self.get_parameter('camera_topic').value, self.img1_cb, 10)
        # Grayscale processed image
        self.create_subscription(Image, 'gray_processed', self.img2_cb, 10)
        
        self.create_subscription(Image, 'prox_processed', self.img3_cb, 10)
        
        self.create_subscription(Image, 'thresh_overlay', self.img4_cb, 10)
        
        #self.create_subscription(Image, '', self.img5_cb, 10)
        
        #self.create_subscription(Image, '', self.img6_cb, 10)
        
        
        #Timer creation
        self.timer = self.create_timer(0.05, self.timer_cb)
        
        #Running message
        self.logger.info("VisualMonitor Initialized!")
        
    def img1_cb(self, msg):
        self.img1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def img2_cb(self, msg):
        self.img2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
    
    def img3_cb(self, msg):
        self.img3 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
    def img4_cb(self, msg):
        self.img4 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def img5_cb(self, msg):
        self.img5 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        
    def img6_cb(self, msg):
        self.img6 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def display_streams(self):
        '''
        Displays all available image streams in a single window arranged in a fixed 3x2 grid.
        Each image is resized to fit its cell without stretching (maintaining its aspect ratio),
        with black letterboxes added and a label in the bottom left corner of each cell.
        '''
        # Collect available image streams.
        streams = []
        if hasattr(self, 'img1'):
            streams.append(self.img1)
        if hasattr(self, 'img2'):
            streams.append(self.img2)
        if hasattr(self, 'img3'):
            streams.append(self.img3)
        if hasattr(self, 'img4'):
            streams.append(self.img4)
        if hasattr(self, 'img5'):
            streams.append(self.img5)
        if hasattr(self, 'img6'):
            streams.append(self.img6)
        
        # To add new streams, simply add additional checks here:
        # if hasattr(self, 'img4'):
        #     streams.append(self.img4)

        if not streams:
            self.logger.warning("No image streams available to display")
            return

        # Define labels for each stream.
        # Edit this list to change labels.
        stream_labels = ["Raw", "Grayscale", "Proximal", "Prox. Overlay", "", ""]

        # Determine cell size from the maximum dimensions of the available images.
        cell_height = max(img.shape[0] for img in streams)
        cell_width  = max(img.shape[1] for img in streams)

        # Define grid dimensions (3 rows by 2 columns).
        grid_rows = 3
        grid_cols = 2
        grid_height = cell_height * grid_rows
        grid_width = cell_width * grid_cols
        canvas = np.zeros((grid_height, grid_width, 3), dtype=np.uint8)

        for idx, img in enumerate(streams):
            # Convert grayscale to BGR if necessary.
            if len(img.shape) == 2 or (len(img.shape) == 3 and img.shape[2] == 1):
                img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            else:
                img_bgr = img

            # Compute scaling factor to fit image in cell without stretching.
            h, w = img_bgr.shape[:2]
            scale = min(cell_width / w, cell_height / h)
            new_w, new_h = int(w * scale), int(h * scale)
            resized = cv2.resize(img_bgr, (new_w, new_h))

            # Create a blank cell and center the resized image.
            cell = np.zeros((cell_height, cell_width, 3), dtype=np.uint8)
            start_x = (cell_width - new_w) // 2
            start_y = (cell_height - new_h) // 2
            cell[start_y:start_y+new_h, start_x:start_x+new_w] = resized

            # Add label text in the bottom left corner with a background box.
            label = stream_labels[idx] if idx < len(stream_labels) else f"Slot {idx+1}"
            font = cv2.FONT_HERSHEY_SIMPLEX

            # Scale font and thickness based on cell height
            font_scale = cell_height / 300.0  # Tune 300 to change base size
            thickness = max(1, int(cell_height / 200))

            # Get text size
            text_size, _ = cv2.getTextSize(label, font, font_scale, thickness)
            text_x = 5
            text_y = cell_height - 5

            # Draw a filled rectangle behind the text for readability.
            cv2.rectangle(
                cell,
                (text_x - 2, text_y - text_size[1] - 2),
                (text_x + text_size[0] + 2, text_y + 2),
                (20, 20, 20),
                thickness=-1
            )

            # Draw the text over the rectangle.
            cv2.putText(
                cell, label, (text_x, text_y),
                font, font_scale, (255, 255, 255),
                thickness, cv2.LINE_AA
            )
            
            # Draw the text over the rectangle.
            cv2.putText(cell, label, (text_x, text_y), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)

            # Position cell in the grid.
            row = idx // grid_cols
            col = idx % grid_cols
            y_start = row * cell_height
            x_start = col * cell_width
            canvas[y_start:y_start+cell_height, x_start:x_start+cell_width] = cell

        # Create and resize window for scalable view.
        cv2.namedWindow('VisualMonitor', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('VisualMonitor', grid_width // 2, grid_height // 2)
        cv2.imshow('VisualMonitor', canvas)
        cv2.waitKey(1)

    def timer_cb(self):
        '''
        Timer callback function.
        Fetches the latest available image streams and displays them.
        '''
        try:
            self.display_streams()
        except Exception as e:
            self.logger.error(f"Error in timer_cb: {e}")
            
    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('ROS time is active!')

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
