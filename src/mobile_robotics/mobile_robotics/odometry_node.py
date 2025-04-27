"""
Aux node for localization
Odometry node for robot localization using wheel encoders
This node subscribes to the left and right wheel encoder topics
and computes the robot's pose (x, y, theta) using odometry equations.
It publishes the pose to the /pose topic.
"""

import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
from rclpy import qos
import numpy as np

class Odometry(Node):

    def __init__(self):
        super().__init__('odometry_node')

        ############## PUBLISHERS ################
        self.pub_pose = self.create_publisher(Pose2D, 'pose', 10)

        ############## SUBSCRIBERS ################
        self.create_subscription(Float32, "VelocityEncR", self.wr_cb, qos.qos_profile_sensor_data)
        self.create_subscription(Float32, "VelocityEncL", self.wl_cb, qos.qos_profile_sensor_data)

        ############## ROBOT CONSTANTS ################
        self.r = 0.05  # Wheel radius [m] -> Will be calibrated if needed
        self.L = 0.19  # Wheel separation [m] -> Will be calibrated if needed

        self.wl = 0.0  # Filtered Left wheel speed [rad/s]
        self.wr = 0.0  # Filtered Right wheel speed [rad/s]

        self.x = 0.0  # Robot x position [m]
        self.y = 0.0  # Robot y position [m]
        self.theta = 0.0  # Robot orientation [rad]

        # Logger config
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.robot_pose = Pose2D()
        self.prev_time_ns = self.get_clock().now().nanoseconds

        timer_period = 0.05  # 20 Hz update
        self.create_timer(timer_period, self.main_timer_cb)
        self.get_logger().info("Odometry Initialized!")

    def main_timer_cb(self):
        now_ns = self.get_clock().now().nanoseconds
        dt = 0.05 #(now_ns - self.prev_time_ns) * 1e-9  # Calculate dt in seconds
        self.prev_time_ns = now_ns  # Update time at the beginning of the callback

        v, w = self.get_robot_velocity(self.wl, self.wr)
        self.update_robot_pose(v, w, dt)

        # Publish the robot's pose
        self.pub_pose.publish(self.robot_pose)

        # Optional: Debug print
        print(f"Pose :)-> x: {self.robot_pose.x:.3f}, y: {self.robot_pose.y:.3f}, theta: {np.degrees(self.robot_pose.theta):.2f}°")

    def wl_cb(self, wl_msg):
        # Apply a simple low-pass filter to reduce noise
        self.wl = 0.8 * self.wl + 0.2 * wl_msg.data

    def wr_cb(self, wr_msg):
        # Apply a simple low-pass filter to reduce noise
        self.wr = 0.8 * self.wr + 0.2 * wr_msg.data

    def get_robot_velocity(self, wl, wr):
        # Calculate linear and angular velocities
        v = self.r * (wr + wl) / 2.0
        w = self.r * (wr - wl) / self.L

        # Optional: Debug print
        print(f"v: {v:.4f} m/s, w: {w:.4f} rad/s")
        return v, w

    def update_robot_pose(self, v, w, dt):
        # Use exact integration for better precision
        if abs(w) < 1e-6:  # If angular velocity is very small (almost straight motion)
            self.x += v * np.cos(self.theta) * dt
            self.y += v * np.sin(self.theta) * dt
        else:  # Robot is turning
            self.x += (v/w) * (np.sin(self.theta + w*dt) - np.sin(self.theta))
            self.y -= (v/w) * (np.cos(self.theta + w*dt) - np.cos(self.theta))
            self.theta += w * dt

        # Normalize theta between [-pi, pi]
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        # Update the pose message
        self.robot_pose.x = self.x
        self.robot_pose.y = self.y
        self.robot_pose.theta = self.theta

def main(args=None):
    rclpy.init(args=args)
    my_node = Odometry()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
