#!/usr/bin/env python3
# my_robot_package/path_generator_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        # Declare node parameters
        self.declare_parameter('path_mode', 'coordinate')
        self.declare_parameter('path_points', [])
        self.declare_parameter('default_yaw', 0.0)
        self.declare_parameter('robust_margin', 0.9)  # Not used for calculations here

        # Read parameters
        self.mode = self.get_parameter('path_mode').value
        self.points = self.get_parameter('path_points').value
        self.default_yaw = self.get_parameter('default_yaw').value

        if not self.points:
            self.get_logger().error("No path points provided. Shutting down.")
            rclpy.shutdown()
            return

        # Create publisher on /pose using standard Float32MultiArray
        self.publisher_ = self.create_publisher(Float32MultiArray, '/pose', 10)
        self.current_index = 0
        self.last_publish_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.current_index >= len(self.points):
            self.get_logger().info("All points published. Stopping timer.")
            self.timer.cancel()
            return

        current_time = self.get_clock().now()
        # Access the current point from the list-of-lists.
        point = self.points[self.current_index]
        # Expect t value at index 2.
        t_interval = point[2]
        elapsed = (current_time - self.last_publish_time).nanoseconds * 1e-9

        if elapsed < t_interval:
            return  # Wait until the specified time interval has passed

        msg = Float32MultiArray()
        if self.mode == "coordinate":
            # Expecting a coordinate point as [x, y, t]
            x = point[0]
            y = point[1]
            msg.data = [x, y, t_interval]
        elif self.mode == "velocity":
            # Expecting a velocity command as [v, w, t]
            v = point[0]
            w = point[1]
            msg.data = [v, w, t_interval]
        else:
            self.get_logger().error("Unknown path_mode specified.")
            self.timer.cancel()
            return

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published point {self.current_index + 1}: {msg.data}")
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