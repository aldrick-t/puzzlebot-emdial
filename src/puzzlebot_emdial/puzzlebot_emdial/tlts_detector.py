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
        self.declare_parameter('tl_conf_threshold', 0.68)  # Traffic light confidence threshold
        self.declare_parameter('ts_conf_threshold', 0.78)  # Traffic sign confidence threshold
        
        self.tl_conf_threshold = self.get_parameter('tl_conf_threshold').get_parameter_value().double_value
        self.ts_conf_threshold = self.get_parameter('ts_conf_threshold').get_parameter_value().double_value
        
        # Bias Parameters for score calculation
        self.declare_parameter('bias_brightness', 0.4)
        self.declare_parameter('bias_size', 0.5)
        self.declare_parameter('bias_position', 0.1)
        
        # Load biases from parameters
        self.biases['brightness'] = self.get_parameter('bias_brightness').get_parameter_value().double_value
        self.biases['size'] = self.get_parameter('bias_size').get_parameter_value().double_value
        self.biases['position'] = self.get_parameter('bias_position').get_parameter_value().double_value
        # Ensure biases sum to 1
        total_bias = sum(self.biases.values())
        if total_bias != 1.0:
            self.get_logger().warn(f'Biases do not sum to 1: {total_bias}. Normalizing biases.')
            for key in self.biases:
                self.biases[key] /= total_bias
        else:
            self.get_logger().info(f'Biases initialized: {self.biases}')
        
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Biases for score calculation (sum must equal 1)
        self.biases = {
            'brightness': 0.4,
            'size': 0.5,
            'position': 0.1
        }

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
            if param.name == 'tl_conf_threshold':
                self.tl_conf_threshold = param.value
                self.get_logger().info(f'Traffic light confidence threshold set to {self.tl_conf_threshold}')
            elif param.name == 'ts_conf_threshold':
                self.ts_conf_threshold = param.value
                self.get_logger().info(f'Traffic sign confidence threshold set to {self.ts_conf_threshold}')
            elif param.name in self.biases:
                self.biases[param.name] = param.value
                self.get_logger().info(f'Bias {param.name} set to {self.biases[param.name]}')
        
        # Normalize biases if they do not sum to 1
        total_bias = sum(self.biases.values())
        if total_bias != 1.0:
            for key in self.biases:
                self.biases[key] /= total_bias
            self.get_logger().warn(f'Biases normalized: {self.biases}')
        
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
        best_tl_score = 0.0

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

                # Traffic light logic
                elif class_name.startswith('tl_'):
                    if conf < self.tl_conf_threshold:
                        continue

                    # Compute size metric (normalized area)
                    w = x2 - x1
                    h = y2 - y1
                    area = w * h
                    size_norm = area / frame_area

                    # Compute position metric (centeredness)
                    centroid_x = (x1 + x2) / 2.0
                    pos_score = 1.0 - abs(centroid_x - frame_center_x) / frame_center_x
                    pos_score = max(0.0, min(1.0, pos_score))

                    # Compute brightness metric (normalized mean intensity)
                    roi = frame[y1:y2, x1:x2]
                    if roi.size > 0:
                        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                        brightness_norm = float(np.mean(gray)) / 255.0
                    else:
                        brightness_norm = 0.0

                    # Compute combined score using biases
                    score = (self.biases['brightness'] * brightness_norm +
                             self.biases['size']       * size_norm +
                             self.biases['position']   * pos_score)

                    # Update best TL candidate
                    if score > best_tl_score:
                        best_tl_score = score
                        best_tl = (class_name, score)

                    # Draw TL bounding box and label with score
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    label = f"{class_name}: {conf:.2f}, score: {score:.2f}"
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
