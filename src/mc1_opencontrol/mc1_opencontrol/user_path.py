#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Empty
from geometry_msgs.msg import Twist
import math

# Define dynamic limits
MAX_LINEAR = 0.6    # m/s
MAX_ANGULAR = 1.5   # rad/s

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('open_loop_controller')
        # Declare parameters
        self.declare_parameter('path_mode', 'coordinate')
        self.mode = self.get_parameter('path_mode').value

        self.declare_parameter('robust_margin', 0.9)
        self.robust_margin = self.get_parameter('robust_margin').value

        # Subscription for pose messages from the path generator.
        self.pose_sub = self.create_subscription(
            Float32MultiArray,
            '/pose',
            self.pose_callback,
            10)

        # Publisher for cmd_vel (to command robot speeds).
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Publisher to trigger the next pose from path_generator.
        self.next_pub = self.create_publisher(Empty, '/next_point', 10)

        # Robot's current pose (assumed known or estimated)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # State variables for the state machine.
        # For coordinate mode: states are "idle", "linear", "rotate".
        # For velocity mode: state "velocity".
        self.state = "idle"
        self.current_target = None  # Dictionary to store target information.
        self.state_start_time = None

        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg: Float32MultiArray):
        data = msg.data
        if self.state != "idle":
            self.get_logger().warn("Already processing a point. Ignoring new pose.")
            return

        if self.mode == "coordinate":
            if len(data) != 3:
                self.get_logger().error("Expected 3 elements for coordinate mode.")
                return
            target_x, target_y, t_duration = data
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)
            if t_duration <= 0:
                self.get_logger().error("t_duration must be > 0.")
                return
            # Compute the linear speed (distance divided by t_duration).
            linear_speed = (distance / t_duration) * self.robust_margin
            linear_speed = min(linear_speed, MAX_LINEAR)
            # Compute desired heading.
            desired_theta = math.atan2(dy, dx)
            dtheta = desired_theta - self.current_theta
            dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
            # Set fixed rotation speed.
            rotation_speed = 0.5
            rotation_duration = abs(dtheta) / rotation_speed

            # Save target info.
            self.current_target = {
                "target_x": target_x,
                "target_y": target_y,
                "t_duration": t_duration,
                "linear_speed": linear_speed,
                "desired_theta": desired_theta,
                "rotation_duration": rotation_duration,
                "rotation_direction": math.copysign(1.0, dtheta)
            }
            self.state = "linear"
            self.state_start_time = self.get_clock().now()
            self.get_logger().info(f"Starting linear phase: Moving to ({target_x}, {target_y}) in {t_duration}s at {linear_speed:.2f} m/s.")

        elif self.mode == "velocity":
            if len(data) != 3:
                self.get_logger().error("Expected 3 elements for velocity mode.")
                return
            v_input, w_input, t_duration = data
            effective_v = v_input * self.robust_margin
            effective_w = w_input * self.robust_margin
            effective_v = min(effective_v, MAX_LINEAR)
            effective_w = math.copysign(min(abs(effective_w), MAX_ANGULAR), effective_w)
            self.current_target = {
                "v": effective_v,
                "w": effective_w,
                "t_duration": t_duration
            }
            self.state = "velocity"
            self.state_start_time = self.get_clock().now()
            self.get_logger().info(f"Starting velocity phase: v={effective_v:.2f} m/s, w={effective_w:.2f} rad/s for {t_duration}s.")

        else:
            self.get_logger().error("Unknown mode in controller.")

    def control_loop(self):
        now = self.get_clock().now()
        if self.state == "idle":
            return

        elapsed = (now - self.state_start_time).nanoseconds * 1e-9
        cmd = Twist()

        if self.mode == "coordinate":
            if self.state == "linear":
                # Command forward motion.
                cmd.linear.x = self.current_target["linear_speed"]
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                if elapsed >= self.current_target["t_duration"]:
                    # Transition to rotation phase.
                    self.state = "rotate"
                    self.state_start_time = now
                    self.get_logger().info("Linear phase complete. Starting rotation phase.")
            elif self.state == "rotate":
                # Command rotation at fixed speed.
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5 * self.current_target["rotation_direction"]
                self.cmd_vel_pub.publish(cmd)
                if elapsed >= self.current_target["rotation_duration"]:
                    # Rotation complete; update current heading.
                    self.current_theta = self.current_target["desired_theta"]
                    self.get_logger().info(f"Rotation complete. New heading: {self.current_theta:.2f} rad.")
                    # Stop the robot.
                    stop_cmd = Twist()
                    self.cmd_vel_pub.publish(stop_cmd)
                    # Trigger the next point.
                    self.next_pub.publish(Empty())
                    self.get_logger().info("Trigger sent for next point.")
                    self.state = "idle"
            else:
                self.get_logger().error("Unknown state in coordinate mode.")
        elif self.mode == "velocity":
            if self.state == "velocity":
                cmd.linear.x = self.current_target["v"]
                cmd.angular.z = self.current_target["w"]
                self.cmd_vel_pub.publish(cmd)
                if elapsed >= self.current_target["t_duration"]:
                    # Duration complete; stop and trigger next point.
                    stop_cmd = Twist()
                    self.cmd_vel_pub.publish(stop_cmd)
                    self.get_logger().info("Velocity phase complete.")
                    self.next_pub.publish(Empty())
                    self.state = "idle"
            else:
                self.get_logger().error("Unknown state in velocity mode.")
        else:
            self.get_logger().error("Unknown mode in control loop.")

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