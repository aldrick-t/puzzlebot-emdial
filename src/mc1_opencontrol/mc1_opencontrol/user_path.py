#!/usr/bin/env python3
# my_robot_package/controller_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import numpy as np

# Define dynamic limits
MAX_LINEAR = 0.6    # m/s
MAX_ANGULAR = 1.5   # rad/s

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('open_loop_controller')
        # Declare the new parameter for path mode.
        self.declare_parameter('path_mode', 'coordinate')
        self.mode = self.get_parameter('path_mode').value

        self.declare_parameter('robust_margin', 0.9)
        self.robust_margin = self.get_parameter('robust_margin').value

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/pose',
            self.pose_callback,
            10)

        # Initialize the robot’s current pose (for coordinate integration)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

    def pose_callback(self, msg: Float32MultiArray):
        data = msg.data

        # For coordinate mode, the message is [x, y, t]
        if self.mode == "coordinate":
            if len(data) != 3:
                self.get_logger().error("Expected 3 elements for coordinate mode.")
                return
            target_x, target_y, t_duration = data
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)
            # Compute desired heading from current pose toward target:
            desired_theta = math.atan2(dy, dx)
            dtheta = desired_theta - self.current_theta

            # Compute speeds based on the provided time
            computed_linear_speed = (distance / t_duration) if t_duration > 0 else 0.0
            computed_angular_speed = (dtheta / t_duration) if t_duration > 0 else 0.0

            # Apply robust margin
            computed_linear_speed *= self.robust_margin
            computed_angular_speed *= self.robust_margin

            # Saturate speeds against maximum allowed limits
            commanded_linear_speed = min(computed_linear_speed, MAX_LINEAR)
            commanded_angular_speed = math.copysign(min(abs(computed_angular_speed), MAX_ANGULAR), computed_angular_speed)

            if computed_linear_speed > MAX_LINEAR or abs(computed_angular_speed) > MAX_ANGULAR:
                self.get_logger().warn(
                    f"[Coord Mode] Target not reachable: Required speeds (linear: {computed_linear_speed:.2f} m/s, angular: {computed_angular_speed:.2f} rad/s) exceed limits.")
            else:
                self.get_logger().info(
                    f"[Coord Mode] Target reachable: Computed speeds (linear: {computed_linear_speed:.2f} m/s, angular: {computed_angular_speed:.2f} rad/s).")
            
            self.get_logger().info(
                f"[Coord Mode] Commanding speeds: linear: {commanded_linear_speed:.2f} m/s, angular: {commanded_angular_speed:.2f} rad/s.")

            # Update current pose (assuming the target is reached after t_duration)
            self.current_x = target_x
            self.current_y = target_y
            self.current_theta = desired_theta

        # For velocity mode, the message is [v, w, t]
        elif self.mode == "velocity":
            if len(data) != 3:
                self.get_logger().error("Expected 3 elements for velocity mode.")
                return
            v_input, w_input, t_duration = data

            # Apply robust margin
            effective_v = v_input * self.robust_margin
            effective_w = w_input * self.robust_margin

            effective_v = min(effective_v, MAX_LINEAR)
            effective_w = math.copysign(min(abs(effective_w), MAX_ANGULAR), effective_w)

            self.get_logger().info(
                f"[Vel Mode] Commanding speeds: linear: {effective_v:.2f} m/s, angular: {effective_w:.2f} rad/s.")
            
            # Euler integration for updated pose:
            delta_x = effective_v * t_duration * math.cos(self.current_theta)
            delta_y = effective_v * t_duration * math.sin(self.current_theta)
            delta_theta = effective_w * t_duration

            self.current_x += delta_x
            self.current_y += delta_y
            self.current_theta += delta_theta

            self.get_logger().info(
                f"[Vel Mode] Updated Pose: x: {self.current_x:.2f}, y: {self.current_y:.2f}, theta: {self.current_theta:.2f} (over t: {t_duration}s)")
        else:
            self.get_logger().error("Unknown mode received in controller.")


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()