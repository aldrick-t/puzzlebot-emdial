import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        self.declare_parameter('path_mode', 'coordinate')
        # Declare with a non-empty default so that the parameter type is properly inferred as DOUBLE_ARRAY.
        self.declare_parameter('path_points', [0.0, 0.0, 0.0])
        self.declare_parameter('robust_margin', 0.9)

        self.mode = self.get_parameter('path_mode').value
        # Get the flat list of numbers from the parameter.
        self.points_flat = self.get_parameter('path_points').value
        self.robust_margin = self.get_parameter('robust_margin').value

        # In coordinate (or velocity) mode, we expect each point to have 3 numbers.
        expected_elements = 3  
        if len(self.points_flat) % expected_elements != 0:
            self.get_logger().error("The number of elements in 'path_points' is not a multiple of {}.".format(expected_elements))
            rclpy.shutdown()
            return

        self.num_points = len(self.points_flat) // expected_elements
        self.current_index = 0
        self.last_publish_time = self.get_clock().now()
        self.publisher_ = self.create_publisher(Float32MultiArray, '/pose', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    # ... rest of the code unchanged ...

    def timer_callback(self):
        if self.current_index >= self.num_points:
            self.get_logger().info("All points published. Stopping timer.")
            self.timer.cancel()
            return

        current_time = self.get_clock().now()
        # Determine the flat list indices for the current point.
        i = self.current_index * 3
        t_interval = self.points_flat[i + 2]
        elapsed = (current_time - self.last_publish_time).nanoseconds * 1e-9

        if elapsed < t_interval:
            return  # Wait until the specified time interval has passed

        msg = Float32MultiArray()

        if self.mode == "coordinate":
            x = self.points_flat[i]
            y = self.points_flat[i + 1]
            msg.data = [x, y, t_interval]  # Only x, y, and t are published
        elif self.mode == "velocity":
            v = self.points_flat[i]
            w = self.points_flat[i + 1]
            msg.data = [v, w, t_interval]
        else:
            self.get_logger().error("Unknown path_mode specified.")
            self.timer.cancel()
            return

        self.publisher_.publish(msg)
        self.get_logger().info("Published point {}: {}".format(self.current_index + 1, msg.data))
        self.last_publish_time = current_time
        self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()