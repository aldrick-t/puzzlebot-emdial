'''
Aux node for localization
Odometry node for robot localization using wheel encoders
This node subscribes to the left and right wheel encoder topics
and computes the robot's pose (x, y, theta) using the odometry equations.
It publishes the pose to the /pose topic.
'''

import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data
import numpy as np

class Odometry(Node):

    def __init__(self):
        super().__init__('odometry_node')
        self.pub_pose = self.create_publisher(Pose2D, 'pose', 10)

        self.create_subscription(Float32, "VelocityEncR", self.wr_cb, qos_profile_sensor_data)
        self.create_subscription(Float32, "VelocityEncL", self.wl_cb, qos_profile_sensor_data)

        self.r = 0.05  # wheel radius for our simulated robot [m]
        self.L = 0.174  # wheel separation for our simulated robot [m]
        self.wl = 0.0  # Left wheel speed [rad/s]
        self.wr = 0.0  # Right wheel speed [rad/s]
        self.x = 0.0  # Robot position in x-axis [m]
        self.y = 0.0  # Robot position in y-axis [m]
        self.theta = 0.0  # Robot orientation [rad]

        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.robot_pose = Pose2D()
        self.prev_time_ns = self.get_clock().now().nanoseconds

        timer_period = 0.05
        self.create_timer(timer_period, self.main_timer_cb)
        self.get_logger().info("Odometry Initialized!")

        # Filter parameters
        self.filter_window_size = 5  # Adjust as needed
        self.wl_filter_buffer = []
        self.wr_filter_buffer = []

    def main_timer_cb(self):
        v, w = self.get_robot_velocity(self.wl, self.wr)
        self.update_robot_pose(v, w)
        print("Robot pose: ", self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta)
        self.pub_pose.publish(self.robot_pose)

    def wl_cb(self, wl):
        self.wl_filter_buffer.append(wl.data)
        if len(self.wl_filter_buffer) > self.filter_window_size:
            self.wl_filter_buffer.pop(0)
        self.wl = np.mean(self.wl_filter_buffer)

    def wr_cb(self, wr):
        self.wr_filter_buffer.append(wr.data)
        if len(self.wr_filter_buffer) > self.filter_window_size:
            self.wr_filter_buffer.pop(0)
        self.wr = np.mean(self.wr_filter_buffer)

    def get_robot_velocity(self, wl, wr):
        v = self.r * (wr + wl) / 2.0
        w = self.r * (wr - wl) / self.L
        return v, w

    def update_robot_pose(self, v, w):
        dt = (self.get_clock().now().nanoseconds - self.prev_time_ns) * 1e-9
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += w * dt
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        self.robot_pose.x = self.x
        self.robot_pose.y = self.y
        self.robot_pose.theta = self.theta
        self.prev_time_ns = self.get_clock().now().nanoseconds

def main(args=None):
    rclpy.init(args=args)
    my_node = Odometry()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
