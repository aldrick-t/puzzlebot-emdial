'''
Square Path Controller Node with PID Control
The robot moves in a square path, turning at each corner.
The robot stops after completing the square path.
Goal points (each corner) are set via params.
'''

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist

class SquarePath(Node):
    def __init__(self):
        super().__init__('square_path')
        self.wait_for_ros_time()

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_sub = self.create_subscription(Twist, 'pose', self.pose_callback, 10)

        # Declare parameters
        self.square_side = self.declare_parameter('square_side', 2.0).get_parameter_value().double_value
        self.robust_margin = self.declare_parameter('robust_margin', 0.9).get_parameter_value().double_value
        self.user_time = self.declare_parameter('user_time', 60).get_parameter_value().double_value