'''
trafficlight_recogni.py

Traffic Light Recognition Node


aldrick-t
MAY 2025
'''

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import SetParametersResult

class TLightRecogni(Node):
    pass