import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Empty
from geometry_msgs.msg import Twist
import math

# Speed and angular limits.
MAX_LINEAR = 0.6      # m/s
MAX_ANGULAR = 1.5     # rad/s  (for velocity mode)
FIXED_ANGULAR_SPEED = 0.7  # rad/s (used in coordinate mode)

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('open_loop_controller')
        # Declare parameters.
        self.declare_parameter('path_mode', 'coordinate')
        self.mode = self.get_parameter('path_mode').value

        self.declare_parameter('robust_margin', 0.9)
        self.robust_margin = self.get_parameter('robust_margin').value

        # Subscribe to /pose messages.
        self.pose_sub = self.create_subscription(
            Float32MultiArray,
            '/pose',
            self.pose_callback,
            10)

        # Publisher for /cmd_vel for commanding motion.
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Publisher for triggering the next segment.
        self.next_pub = self.create_publisher(Empty, '/next_point', 10)

        # Robot's current estimated position and orientation.
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # State machine variables.
        # Valid states: "idle", "rotate", "linear", or "velocity" (when in one phase)
        self.state = "idle"
        self.current_target = None  # To store target details and phase durations.
        self.state_start_time = None

        # Timer for the control loop.
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg: Float32MultiArray):
        data = msg.data
        # Do not accept new points until processing of the current one is finished.
        if self.state != "idle":
            self.get_logger().warn("Already processing a point. Ignoring new pose.")
            return

        # --- Coordinate Mode (already working as desired) ---
        if self.mode == "coordinate":
            if len(data) != 3:
                self.get_logger().error("Coordinate mode requires 3 values: x, y, and t.")
                return
            target_x, target_y, t_duration = data
            if t_duration <= 0:
                self.get_logger().error("t_duration must be > 0.")
                return

            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)
            
            # Determine desired heading.
            desired_theta = math.atan2(dy, dx)
            dtheta = desired_theta - self.current_theta
            dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
            # Rotation phase duration computed from the fixed angular speed.
            rotation_duration = abs(dtheta) / FIXED_ANGULAR_SPEED
            rotation_direction = math.copysign(1.0, dtheta) if dtheta != 0 else 0.0

            # Compute linear speed.
            linear_speed = (distance / t_duration) * self.robust_margin
            linear_speed = min(linear_speed, MAX_LINEAR)

            # Store target information.
            self.current_target = {
                "target_x": target_x,
                "target_y": target_y,
                "t_duration": t_duration,
                "linear_speed": linear_speed,
                "desired_theta": desired_theta,
                "rotation_duration": rotation_duration,
                "rotation_direction": rotation_direction
            }
            # If no rotation is needed, skip to the linear phase.
            if rotation_duration <= 0.0:
                self.get_logger().info("No rotation needed. Starting linear phase.")
                self.state = "linear"
            else:
                self.state = "rotate"
                self.get_logger().info(
                    f"Starting rotation phase: Rotate by {dtheta:.2f} rad (duration: {rotation_duration:.2f}s) at {FIXED_ANGULAR_SPEED:.2f} rad/s."
                )
            self.state_start_time = self.get_clock().now()

        # --- Velocity Mode (now split into rotate then linear) ---
        elif self.mode == "velocity":
            if len(data) != 3:
                self.get_logger().error("Velocity mode requires 3 values: v, w, and t.")
                return
            v_input, w_input, t_duration = data
            if t_duration <= 0:
                self.get_logger().error("t_duration must be > 0.")
                return
            
            # Calculate effective velocities (applying robust margin and caps).
            effective_v = min(v_input * self.robust_margin, MAX_LINEAR)
            effective_w = math.copysign(min(abs(w_input * self.robust_margin), MAX_ANGULAR), w_input)
            
            # For velocity mode we split the total duration into two phases.
            # If an angular command is provided, use half of the time for rotation.
            if abs(effective_w) > 1e-6:
                rotation_duration = t_duration / 2.0
                linear_duration = t_duration - rotation_duration
                # Compute what the new heading will be after rotation.
                desired_theta = self.current_theta + effective_w * rotation_duration
            else:
                rotation_duration = 0.0
                linear_duration = t_duration
                desired_theta = self.current_theta

            self.current_target = {
                "v": effective_v,
                "w": effective_w,
                "rotation_duration": rotation_duration,
                "linear_duration": linear_duration,
                "desired_theta": desired_theta
            }
            
            if rotation_duration > 0.0:
                self.state = "rotate"
                self.get_logger().info(
                    f"Starting velocity mode rotation phase: rotating for {rotation_duration:.2f}s with angular velocity "
                    f"{effective_w:.2f} rad/s to achieve heading {desired_theta:.2f} rad."
                )
            else:
                self.state = "linear"
                self.get_logger().info("No rotation commanded. Starting velocity mode linear phase.")
            self.state_start_time = self.get_clock().now()
        else:
            self.get_logger().error("Unknown mode in controller.")

    def control_loop(self):
        now = self.get_clock().now()
        if self.state == "idle":
            return

        elapsed = (now - self.state_start_time).nanoseconds * 1e-9
        cmd = Twist()

        # --- Coordinate Mode control ---
        if self.mode == "coordinate":
            if self.state == "rotate":
                if elapsed < self.current_target["rotation_duration"]:
                    cmd.angular.z = FIXED_ANGULAR_SPEED * self.current_target["rotation_direction"]
                    cmd.linear.x = 0.0
                    self.cmd_vel_pub.publish(cmd)
                else:
                    # Rotation complete.
                    self.current_theta = self.current_target["desired_theta"]
                    self.get_logger().info(f"Rotation complete. New heading: {self.current_theta:.2f} rad.")
                    self.cmd_vel_pub.publish(Twist())  # Stop
                    self.state = "linear"
                    self.state_start_time = now
            elif self.state == "linear":
                if elapsed < self.current_target["t_duration"]:
                    cmd.linear.x = self.current_target["linear_speed"]
                    cmd.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd)
                else:
                    # Linear movement complete.
                    self.current_x = self.current_target["target_x"]
                    self.current_y = self.current_target["target_y"]
                    self.get_logger().info(
                        f"Linear movement complete. New position: ({self.current_x:.2f}, {self.current_y:.2f})."
                    )
                    self.cmd_vel_pub.publish(Twist())  # Stop
                    self.next_pub.publish(Empty())
                    self.state = "idle"
            else:
                self.get_logger().error("Unknown state in coordinate mode.")

        # --- Velocity Mode control ---
        elif self.mode == "velocity":
            if self.state == "rotate":
                if elapsed < self.current_target["rotation_duration"]:
                    cmd.angular.z = self.current_target["w"]
                    cmd.linear.x = 0.0
                    self.cmd_vel_pub.publish(cmd)
                else:
                    # Rotation phase complete.
                    self.current_theta = self.current_target["desired_theta"]
                    self.get_logger().info(f"Rotation phase complete in velocity mode. New heading: {self.current_theta:.2f} rad.")
                    self.cmd_vel_pub.publish(Twist())  # Stop rotation.
                    self.state = "linear"
                    self.state_start_time = now
            elif self.state == "linear":
                if elapsed < self.current_target["linear_duration"]:
                    cmd.linear.x = self.current_target["v"]
                    cmd.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd)
                else:
                    # Linear phase complete: update position by integrating the travel distance.
                    linear_duration = self.current_target["linear_duration"]
                    distance = self.current_target["v"] * linear_duration
                    self.current_x += distance * math.cos(self.current_theta)
                    self.current_y += distance * math.sin(self.current_theta)
                    self.get_logger().info(
                        f"Linear phase complete in velocity mode. New position: ({self.current_x:.2f}, {self.current_y:.2f})."
                    )
                    self.cmd_vel_pub.publish(Twist())  # Stop.
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