#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Empty

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        # Declare parameters
        self.declare_parameter('path_mode', 'coordinate')
        # Use a nonempty default so that the parameter type is clearly DOUBLE_ARRAY.
        self.declare_parameter('path_points', [0.0, 0.0, 0.0])
        self.declare_parameter('robust_margin', 0.9)

        self.mode = self.get_parameter('path_mode').value
        self.points_flat = self.get_parameter('path_points').value
        self.robust_margin = self.get_parameter('robust_margin').value

        expected_elements = 3  # For both coordinate ([x, y, t]) and velocity ([v, w, t]) modes.
        if len(self.points_flat) % expected_elements != 0:
            self.get_logger().error("The number of elements in 'path_points' is not a multiple of {}.".format(expected_elements))
            rclpy.shutdown()
            return
        self.num_points = len(self.points_flat) // expected_elements
        self.current_index = 0

        # Publisher: publishes pose messages on /pose.
        self.pose_pub = self.create_publisher(Float32MultiArray, '/pose', 10)
        # Subscriber: listens for trigger messages on /next_point.
        self.trigger_sub = self.create_subscription(Empty, '/next_point', self.trigger_callback, 10)

        self.get_logger().info("PathGenerator initialized.")
        # Immediately publish the first pose if available.
        self.publish_point(initial=True)

    def publish_point(self, initial=False):
        if self.current_index >= self.num_points:
            self.get_logger().info("All points have been published. No more points.")
            return

        i = self.current_index * 3
        t_interval = self.points_flat[i + 2]
        msg_out = Float32MultiArray()
        if self.mode == "coordinate":
            x = self.points_flat[i]
            y = self.points_flat[i + 1]
            msg_out.data = [x, y, t_interval]
        elif self.mode == "velocity":
            v = self.points_flat[i]
            w = self.points_flat[i + 1]
            msg_out.data = [v, w, t_interval]
        else:
            self.get_logger().error("Unknown path_mode specified.")
            return

        self.pose_pub.publish(msg_out)
        if initial:
            self.get_logger().info("Published initial point {}: {}".format(self.current_index + 1, msg_out.data))
        else:
            self.get_logger().info("Published point {}: {}".format(self.current_index + 1, msg_out.data))
        self.current_index += 1

    def trigger_callback(self, msg):
        # When a trigger message is received, publish the next point if available.
        self.publish_point()

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()