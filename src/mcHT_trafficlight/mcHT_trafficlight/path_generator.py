'''
Path Generator Node, publishes goal points to the /goal topic.
Receives all goal points from user defined parameters.
The path is defined in a 2D coordinate system by Pose2D messages.
The path is defined by a list of x,y coordinates.
Theta in Pose2D is set to 0.0 by default.
Receives trigger messages on /next_goal to publish the next point.
'''

import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D
from rcl_interfaces.msg import SetParametersResult


class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        # Logger config
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Declare and load parameters
        self.raw = self.declare_parameter('path_points', [0.0, 0.0]).value
        self.points = self._parse_points(self.raw)

        if not self.points:
            self.get_logger().error('No valid path points specified!')

        # Register parameter callback ONCE here
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Initialize index
        self.index = -1

        # Publisher & subscriber
        self.goal_pub = self.create_publisher(Pose2D, 'goal', 10)
        self.create_subscription(Empty, 'next_goal', self._next_goal_cb, 10)

        self.get_logger().info("Path Generator Initialized")
        self.get_logger().debug(f'Initial path points: {self.points}')
        self.get_logger().debug("Waiting for /next_goal messages...")

    def _parse_points(self, raw):
        if len(raw) % 2 != 0:
            self.get_logger().fatal('path_points must have an even number of elements (x,y pairs)')
            return []
        return [[raw[i], raw[i+1]] for i in range(0, len(raw), 2)]

    def _next_goal_cb(self, msg):
        if not self.points:
            self.get_logger().warn("No path points defined. Ignoring /next_goal trigger.")
            return

        if self.index + 1 >= len(self.points):
            self.get_logger().info('Reached end of path. No more points to publish.')
            return

        self.index += 1
        self._publish(self.index)

    def _publish(self, idx):
        point = self.points[idx]

        msg = Pose2D()
        msg.x = point[0]
        msg.y = point[1]
        msg.theta = 0.0  # Default theta

        self.goal_pub.publish(msg)
        self.get_logger().info(f'Publishing point #{idx}: {point}')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'path_points':
                if param.value != self.raw:
                    self.get_logger().info(f'Updating path points to: {param.value}')
                    self.raw = param.value
                    self.points = self._parse_points(self.raw)
                    if self.points:
                        self.index = -1  # Restart index if points changed
                        self.get_logger().debug(f'Updated path points: {self.points}')
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
