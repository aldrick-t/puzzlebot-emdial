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
        self.declare_parameter('xv_1', 0.0)
        self.declare_parameter('yw_1', 0.0)
        self.declare_parameter('t_1', 0.0)
        self.declare_parameter('xv_2', 0.0)
        self.declare_parameter('yw_2', 0.0)
        self.declare_parameter('t_2', 0.0)
        self.declare_parameter('xv_3', 0.0)
        self.declare_parameter('yw_3', 0.0)
        self.declare_parameter('t_3', 0.0)
        self.declare_parameter('xv_4', 0.0)
        self.declare_parameter('yw_4', 0.0)
        self.declare_parameter('t_4', 0.0)
        self.declare_parameter('robust_margin', 0.9)  # Not used for calculations here

        # Read parameters
        self.mode = self.get_parameter('path_mode').value
        self.points = [ 
            {'x': self.get_parameter('xv_1').value, 'y': self.get_parameter('yw_1').value, 't': self.get_parameter('t_1').value},
            {'x': self.get_parameter('xv_2').value, 'y': self.get_parameter('yw_2').value, 't': self.get_parameter('t_2').value},
            {'x': self.get_parameter('xv_3').value, 'y': self.get_parameter('yw_3').value, 't': self.get_parameter('t_3').value},
            {'x': self.get_parameter('xv_4').value, 'y': self.get_parameter('yw_4').value, 't': self.get_parameter('t_4').value}
        ]

        self.velocity_points = [
            {'v': self.get_parameter('xv_1').value, 'w': self.get_parameter('yw_1').value, 't': self.get_parameter('t_1').value},
            {'v': self.get_parameter('xv_2').value, 'w': self.get_parameter('yw_2').value, 't': self.get_parameter('t_2').value},
            {'v': self.get_parameter('xv_3').value, 'w': self.get_parameter('yw_3').value, 't': self.get_parameter('t_3').value},
            {'v': self.get_parameter('xv_4').value, 'w': self.get_parameter('yw_4').value, 't': self.get_parameter('t_4').value}
        ]
        # Check if path_mode is valid
        if self.mode not in ["coordinate", "velocity"]:
            self.get_logger().error("Invalid path_mode specified. Use 'coordinate' or 'velocity'.")
            rclpy.shutdown()
            return
        
        # Check if path points are provided
        if self.mode == "coordinate":
            self.points = [point for point in self.points if 'x' in point and 'y' in point and 't' in point]
        elif self.mode == "velocity":
            self.points = [point for point in self.velocity_points if 'v' in point and 'w' in point and 't' in point]
        # Check if path points are empty    
        if not self.points:
            self.get_logger().error("No path points provided. Shutting down.")
            rclpy.shutdown()
            return
        # Check if path points are valid
        for point in self.points:
            if 't' not in point or point['t'] <= 0:
                self.get_logger().error("Invalid time interval (t) specified. Must be positive.")
                rclpy.shutdown()
                return
        # Check if path points are valid
        for point in self.points:
            if 'x' not in point or 'y' not in point:
                self.get_logger().error("Invalid path points specified. Must contain 'x' and 'y'.")
                rclpy.shutdown()
                return
        # Check if path points are valid
        for point in self.points:
            if 'v' not in point or 'w' not in point:
                self.get_logger().error("Invalid path points specified. Must contain 'v' and 'w'.")
                rclpy.shutdown()
                return
            

        if not self.points:
            self.get_logger().error("No path points provided. Shutting down.")
            rclpy.shutdown()
            return

        # Create 4 publishers on /pose topic
        # Create a publisher for the path points that will be sent to the robot each t_i 

        self.publisher = self.create_publisher(Float32, '/pose', 10)



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
        point = self.points[self.current_index]
        t_interval = point.get('t', 1.0)
        elapsed = (current_time - self.last_publish_time).nanoseconds * 1e-9

        if elapsed < t_interval:
            return  # Wait until the specified time interval has passed

        msg = Float32MultiArray()
        if self.mode == "coordinate":
            # Expecting a coordinate point: {x, y, optionally yaw, and t}
            x = point.get('x', 0.0)
            y = point.get('y', 0.0)
            yaw = point.get('yaw', self.default_yaw)
            # Build message: [x, y, yaw, t]
            msg.data = [x, y, yaw, t_interval]
        elif self.mode == "velocity":
            # Expecting a velocity command: {v, w, t}
            v = point.get('v', 0.0)
            w = point.get('w', 0.0)
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