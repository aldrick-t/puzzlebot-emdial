import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist

class UserParameter(Node):
    def __init__(self):
        super().__init__('user_path')
        self.wait_for_ros_time()

        self.pose_pub = self.create_publisher(Twist, 'pose', 10)

        # Declare parameters
        self.square_side = self.declare_parameter('square_side', 2.0).get_parameter_value().double_value
        self.robust_margin = self.declare_parameter('robust_margin', 0.9).get_parameter_value().double_value
        self.user_time = self.declare_parameter('user_time', 60).get_parameter_value().double_value
        self.user_linear_speed = self.declare_parameter('user_linear_speed', 0.3).get_parameter_value().double_value
        self.user_angular_speed = self.declare_parameter('user_angular_speed', 0.3).get_parameter_value().double_value
        self.turn_angle = self.declare_parameter('turn_angle', np.pi/2).get_parameter_value().double_value

        # Parameter validation
        if self.user_time and (self.user_linear_speed or self.user_angular_speed):
            self.get_logger().error("Provide either 'user_time' or 'user_linear_speed' + 'user_angular_speed', not both.")
            raise ValueError("Invalid parameter combination.")

        if not self.user_time and (self.user_linear_speed is None or self.user_angular_speed is None):
            self.get_logger().error("When 'user_time' is not set, both 'user_linear_speed' and 'user_angular_speed' must be defined.")
            raise ValueError("Missing required parameters.")

        self.auto_tune()

        self.state = 0
        self.leg_counter = 0
        self.state_start_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Square Path Controller Initialized')

    def auto_tune(self):
        if self.user_time:
            travel_time = self.user_time * 0.8
            rotate_time = self.user_time * 0.2

            self.linear_speed = (self.square_side / (travel_time / 4)) * self.robust_margin
            self.forward_time = self.square_side / self.linear_speed

            self.angular_speed = (self.turn_angle / (rotate_time / 4)) * self.robust_margin
            self.rotate_time = self.turn_angle / self.angular_speed
        else:
            self.linear_speed = self.user_linear_speed * self.robust_margin
            self.angular_speed = self.user_angular_speed * self.robust_margin

            self.forward_time = self.square_side / self.linear_speed
            self.rotate_time = self.turn_angle / self.angular_speed

        self.get_logger().info(f"Linear speed: {self.linear_speed:.2f} m/s, time: {self.forward_time:.2f} s")
        self.get_logger().info(f"Angular speed: {self.angular_speed:.2f} rad/s, time: {self.rotate_time:.2f} s")

    
    def control_loop(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.state_start_time).nanoseconds * 1e-9
        cmd = Twist()

        if self.state == 0:
            cmd.linear.x = self.linear_speed
            if elapsed_time >= self.forward_time:
                self.state = 1
                self.state_start_time = now
                self.get_logger().info(f"[SIDE {self.leg_counter + 1}] Forward complete. Turning...")

        elif self.state == 1:
            cmd.angular.z = self.angular_speed
            self.get_logger().info(f"Current angular speed: {cmd.angular.z:.2f} rad/s")
            if elapsed_time >= self.rotate_time:
                self.leg_counter += 1
                if self.leg_counter >= 4:
                    self.state = 2
                    self.get_logger().info(f"[TURN {self.leg_counter}] Final turn complete.")
                else:
                    self.state = 0
                    self.get_logger().info(f"[TURN {self.leg_counter}] Turn complete. Moving forward...")
                self.state_start_time = now

        elif self.state == 2:
            self.get_logger().info("🛑 Square path complete. Stopping robot.")
            self.pose_pub.publish(Twist())
            self.timer.cancel()
            return

        self.pose_pub.publish(cmd)

    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('ROS time is active!')

def main(args=None):
    rclpy.init(args=args)
    node = UserParameter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()