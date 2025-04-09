# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

# Class Definition
class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('open_loop_ctrl')

        self.wait_for_ros_time()

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Default parameters
        self.square_side = 2.0  # meters
        self.turn_angle = np.pi / 2  # 90 degrees per corner
        self.robust_margin = 0.9 # 10% buffer for robustness

        # User-defined input: either speed or time
        self.user_speed = None  # m/s or None
        self.user_time = 50 # seconds or None

        # Auto-tune parameters
        self.auto_tune()

        # Internal state
        self.state = 0
        self.leg_counter = 0
        self.state_start_time = self.get_clock().now()

        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info('Open Loop Controller Initialized!')

    def auto_tune(self):
        # User defines time or speed

        if self.user_time:
            # Distribute time
            travel_time = self.user_time * 0.7
            rotate_time = self.user_time * 0.3

            self.linear_speed = (self.square_side / (travel_time / 4)) * self.robust_margin
            self.forward_time = self.square_side / self.linear_speed

            self.angular_speed = (self.turn_angle / (rotate_time / 4)) * self.robust_margin
            self.rotate_time = self.turn_angle / self.angular_speed
            self.get_logger().info(f"Calculated angular speed: {self.angular_speed:.2f} rad/s, rotate time: {self.rotate_time:.2f} s")
        else:
            self.linear_speed = self.user_speed * self.robust_margin
            self.forward_time = self.square_side / self.linear_speed

            self.angular_speed = 0.3  # default
            self.rotate_time = self.turn_angle / self.angular_speed


    def control_loop(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.state_start_time).nanoseconds * 1e-9

        self.get_logger().info(f"Start: {self.state_start_time.nanoseconds * 1e-9}, NOW: {now.nanoseconds * 1e-9:.2f}s")
        self.get_logger().info(f"State: {self.state}, Elapsed: {elapsed_time:.2f}s")

        cmd = Twist()

        if self.state == 0:
            # Move forward
            cmd.linear.x = self.linear_speed
            #self.get_logger().info(f"[SIDE {self.leg_counter + 1}/4] → Moving forward: {elapsed_time:.2f}s / {self.forward_time:.2f}s")
            if elapsed_time >= self.forward_time:
                self.state = 1
                self.state_start_time = now
                self.get_logger().info(f"[SIDE {self.leg_counter + 1}] ✅ Forward complete. Preparing to turn 90°...")

        elif self.state == 1:
            # Turn 90 degrees
            cmd.angular.z = self.angular_speed
            #self.get_logger().info(f"[TURN {self.leg_counter + 1}/4] ↻ Rotating: {elapsed_time:.2f}s / {self.rotate_time:.2f}s")
            self.get_logger().info(f"Current angular speed: {cmd.angular.z:.2f} rad/s")
            if elapsed_time >= self.rotate_time:
                self.leg_counter += 1
                if self.leg_counter >= 4:
                    self.state = 2  # All sides complete
                    self.get_logger().info(f"[TURN {self.leg_counter}] ✅ Final turn complete.")
                else:
                    self.state = 0  # Go back to move forward
                    self.get_logger().info(f"[TURN {self.leg_counter}] ✅ Turn complete. Moving forward next...")
                self.state_start_time = now

        elif self.state == 2:
            # Stop
            self.get_logger().info("🛑 Square path complete. Stopping robot.")
            self.cmd_vel_pub.publish(Twist())  # Send stop command
            self.timer.cancel()
            return

        # Publish command to robot
        self.cmd_vel_pub.publish(cmd)

    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('ROS time is active!')

# Main function
def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopCtrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# Execute Node
if __name__ == '__main__':
    main()