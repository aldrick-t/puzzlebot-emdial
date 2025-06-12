#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
class TLTSDetector(Node):
    def __init__(self):
        super().__init__('tlts_detector')
        self.bridge = CvBridge()

        # Create a named window for display
        self.window_name = 'Traffic Light Detection'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
        # Declare parameters for detection thresholds
        self.declare_parameter('ts_conf_threshold', 0.62)  # Traffic sign confidence threshold
        self.declare_parameter('tl_green_threshold', 0.6)  # Traffic light confidence threshold
        self.declare_parameter('tl_yellow_threshold', 0.2)  # Traffic light yellow confidence threshold
        self.declare_parameter('tl_red_threshold', 0.45)  # Traffic light yellow confidence threshold

        
        self.ts_conf_threshold = self.get_parameter('ts_conf_threshold').get_parameter_value().double_value
        self.tl_green_threshold = self.get_parameter('tl_green_threshold').get_parameter_value().double_value
        self.tl_yellow_threshold = self.get_parameter('tl_yellow_threshold').get_parameter_value().double_value
        self.tl_red_threshold = self.get_parameter('tl_red_threshold').get_parameter_value().double_value


        
        self.add_on_set_parameters_callback(self.parameter_callback)


        # Subscribe to camera feed
        self.create_subscription(
            Image,
            '/video_source/raw',
            self.image_callback,
            10
        )

        # Publishers for detected classes
        self.publisher_tl = self.create_publisher(
            String,
            '/traffic_light',
            10
        )
        self.publisher_ts = self.create_publisher(
            String,
            '/traffic_sign',
            10
        )

        # Load custom-trained YOLOv8 model
        self.model = YOLO('./src/puzzlebot_emdial/models/jun5_v8v8_e_uni_tlts.pt')

        # Map class indices → names
        self.class_map = {
            0: 'tl_green',
            1: 'tl_red',
            2: 'tl_yellow',
            3: 'ts_giveway',
            4: 'ts_left',
            5: 'ts_right',
            6: 'ts_stop',
            7: 'ts_straight',
            8: 'ts_work'
        }
        # BGR colors for drawing
        self.colors = {
            'tl_red':      (0,   0, 255),
            'tl_yellow':   (0, 255, 255),
            'tl_green':    (0, 255,   0),
            'ts_giveway':  (255,255,255),
            'ts_left':     (100,255,255),
            'ts_right':    (255,100,255),
            'ts_stop':     (255,255,100),
            'ts_straight': (100,255,100),
            'ts_work':     (100,100,255),
            'unknown':     (255,255,255),
        }
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'tl_green_threshold':
                self.tl_green_threshold = param.value
                self.get_logger().info(f'Traffic light green threshold set to {self.tl_green_threshold}', throttle_duration_sec=1.0)
            elif param.name == 'tl_yellow_threshold':
                self.tl_yellow_threshold = param.value
                self.get_logger().info(f'Traffic light yellow threshold set to {self.tl_yellow_threshold}', throttle_duration_sec=1.0)
            elif param.name == 'tl_red_threshold':
                self.tl_red_threshold = param.value
                self.get_logger().info(f'Traffic light red threshold set to {self.tl_red_threshold}', throttle_duration_sec=1.0)
            elif param.name == 'ts_conf_threshold':
                self.ts_conf_threshold = param.value
                self.get_logger().info(f'Traffic sign confidence threshold set to {self.ts_conf_threshold}', throttle_duration_sec=1.0)

        
        return SetParametersResult(successful=True)
    

    def image_callback(self, img_msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        height, width = frame.shape[:2]
        frame_center_x = width / 2.0
        frame_area = width * height

        # Run detection
        results = self.model(frame)[0]
        boxes     = results.boxes.xyxy.cpu().numpy()
        confs     = results.boxes.conf.cpu().numpy()
        class_ids = results.boxes.cls.cpu().numpy().astype(int)

        # Track best detections
        best_ts = ('none', 0.0)
        best_ts_conf = 0.0
        best_tl = ('none', 0.0)
        best_tl_conf = 0.0  # used to track best tl_ detection

        if boxes.shape[0] > 0:
            for box, conf, cls_id in zip(boxes, confs, class_ids):
                class_name = self.class_map.get(cls_id, 'unknown')
                color = self.colors.get(class_name, self.colors['unknown'])
                x1, y1, x2, y2 = map(int, box)

                # Traffic sign logic
                if class_name.startswith('ts_'):
                    if conf < self.ts_conf_threshold:
                        continue
                    if conf > best_ts_conf:
                        best_ts_conf = conf
                        best_ts = (class_name, conf)

                    # Draw TS bounding box and label
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    label = f"{class_name}: {conf:.2f}"
                    cv2.putText(frame, label, (x1, max(y1 - 10, 0)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # Traffic light logic for each tl_ class separately
                elif class_name == 'tl_red':
                    if conf < self.tl_red_threshold:
                        continue
                    if conf > best_tl_conf:
                        best_tl_conf = conf
                        best_tl = (class_name, conf)

                elif class_name == 'tl_yellow':
                    if conf < self.tl_yellow_threshold:
                        continue
                    if conf > best_tl_conf:
                        best_tl_conf = conf
                        best_tl = (class_name, conf)

                elif class_name == 'tl_green':
                    if conf < self.tl_green_threshold:
                        continue
                    if conf > best_tl_conf:
                        best_tl_conf = conf
                        best_tl = (class_name, conf)

                # Draw TL bounding box and label if it was any tl_ class
                if class_name.startswith('tl_') and conf >= 0.01:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    label = f"{class_name}: {conf:.2f}"
                    cv2.putText(frame, label, (x1, max(y1 - 10, 0)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        else:
            cv2.putText(frame, 'No detection', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # Display
        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

        # Publish results (only class names)
        ts_msg = String(data=best_ts[0])
        tl_msg = String(data=best_tl[0])
        self.publisher_ts.publish(ts_msg)
        self.publisher_tl.publish(tl_msg)
        self.get_logger().info(f'Published TS: "{ts_msg.data}"', throttle_duration_sec=1.0)
        self.get_logger().info(f'Published TL: "{tl_msg.data}"', throttle_duration_sec=1.0)     


def main(args=None):
    rclpy.init(args=args)
    node = TLTSDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
