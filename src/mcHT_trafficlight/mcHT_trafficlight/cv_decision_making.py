#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.logging import LoggingSeverity

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector')
        # --- DEBUG logging ---
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.get_logger().debug('Initializing ColorDetectorNode')

        # --- declare tunable parameters ---
        self.declare_parameter('min_area', 300)
        default_ranges = {
            'RED1':   {'low': [0,100,100],   'high': [10,255,255]},
            'RED2':   {'low': [160,100,100], 'high': [180,255,255]},
            'YELLOW': {'low': [18,100,100],  'high': [30,255,255]},
            'GREEN':  {'low': [40,40,40],    'high': [80,255,255]},
        }
        for name, bounds in default_ranges.items():
            self.declare_parameter(f'hsv_ranges.{name}.low',  bounds['low'])
            self.declare_parameter(f'hsv_ranges.{name}.high', bounds['high'])

        # load into local storage
        self._load_params()

        # watch for dynamic changes
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # --- CV / ROS setup ---
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/video_source/raw', self.camera_callback, 10
        )
        self.img_pub = self.create_publisher(Image,  'processed_img',        10)
        self.col_pub = self.create_publisher(String, 'traffic_light_color', 10)

        # OpenCV windows
        cv2.namedWindow('Contours', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Swatches', cv2.WINDOW_NORMAL)
        self.swatch_img = self._make_swatch_image()

    def _load_params(self):
        # reload min_area
        self.min_area = self.get_parameter('min_area').value
        self.get_logger().debug(f'Loaded parameter min_area = {self.min_area}')

        # reload HSV ranges
        self.hsv_ranges = {}
        for name in ['RED1','RED2','YELLOW','GREEN']:
            low  = self.get_parameter(f'hsv_ranges.{name}.low').value
            high = self.get_parameter(f'hsv_ranges.{name}.high').value
            self.hsv_ranges[name] = {
                'low':  np.array(low,  dtype=np.uint8),
                'high': np.array(high, dtype=np.uint8)
            }
            self.get_logger().debug(f'Loaded {name} → low={low}, high={high}')

    def _on_parameter_change(self, params):
        # any change to our declared params triggers reload
        self.get_logger().debug('Parameter change detected, reloading...')
        try:
            self._load_params()
            self.swatch_img = self._make_swatch_image()
            self.get_logger().debug('Parameters reloaded successfully')
            return SetParametersResult(successful=True)
        except Exception as e:
            self.get_logger().error(f'Failed to reload parameters: {e}')
            return SetParametersResult(successful=False)

    def camera_callback(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        color, out_img, avg_bgr, avg_hsv = self._detect_color(cv_img)

        # publish processed image
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(out_img, 'bgr8'))

        if color:
            self.col_pub.publish(String(data=color))
            self.get_logger().debug(
                f'Detected {color} → avg BGR={tuple(int(x) for x in avg_bgr)}, '
                f'avg HSV={tuple(int(x) for x in avg_hsv)}'
            )
        else:
            self.get_logger().debug('No color detected in this frame')

        # show windows
        cv2.imshow('Contours', out_img)
        cv2.imshow('Swatches', self.swatch_img)
        cv2.waitKey(1)

    def _detect_color(self, img: np.ndarray):
        self.get_logger().debug('Running _detect_color()')
        small = cv2.resize(img, (320,240))
        blur  = cv2.GaussianBlur(small, (5,5), 0)
        hsv   = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # iterate through each named range
        for name, bounds in self.hsv_ranges.items():
            low, high = bounds['low'], bounds['high']
            self.get_logger().debug(f'Checking {name}: low={low.tolist()}, high={high.tolist()}')

            mask = cv2.inRange(hsv, low, high)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self.get_logger().debug(f'  Found {len(cnts)} contours for {name}')

            for cnt in cnts:
                area = cv2.contourArea(cnt)
                self.get_logger().debug(f'    Contour area = {area:.1f}')
                if area > self.min_area:
                    self.get_logger().debug(f'    Area > min_area ({self.min_area}), selecting {name}')
                    cv2.drawContours(small, [cnt], -1, (0,0,0), 2)

                    # compute average BGR inside the contour
                    mask_fill = np.zeros(mask.shape, np.uint8)
                    cv2.drawContours(mask_fill, [cnt], -1, 255, -1)
                    avg_bgr = cv2.mean(small, mask=mask_fill)[:3]
                    avg_hsv = cv2.cvtColor(
                        np.uint8([[avg_bgr]]), cv2.COLOR_BGR2HSV
                    )[0,0]

                    # normalize name to RED/YELLOW/GREEN
                    simple = name.replace('1','').replace('2','')
                    return simple, small, avg_bgr, avg_hsv

        # nothing detected
        return None, small, (0,0,0), (0,0,0)

    def _make_swatch_image(self):
        h, w = 50, 300
        canvas = np.zeros((h*len(self.hsv_ranges), w, 3), np.uint8)
        for i, (name, b) in enumerate(self.hsv_ranges.items()):
            y0 = i*h
            half = w//2
            low_bgr  = cv2.cvtColor(np.uint8([[b['low']]]),  cv2.COLOR_HSV2BGR)[0,0]
            high_bgr = cv2.cvtColor(np.uint8([[b['high']]]), cv2.COLOR_HSV2BGR)[0,0]
            canvas[y0:y0+h, 0:half]     = low_bgr
            canvas[y0:y0+h, half:w]     = high_bgr
            cv2.putText(canvas, name, (5, y0 + h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        return canvas

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()