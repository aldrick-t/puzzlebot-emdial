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
        self.get_logger().debug("Logger set to DEBUG level")
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Declare path_points as a comma-separated string parameter with a descriptor for dynamic reconfigure.
        from rcl_interfaces.msg import ParameterDescriptor
        pts_descriptor = ParameterDescriptor(description="Comma-separated list of x,y coordinates (even number of elements). Example: '1.2,0.0,1.2,1.2'")
        path_points_str = self.declare_parameter('path_points', "0.0,0.0", pts_descriptor).value

        try:
            self.raw = [float(x.strip()) for x in path_points_str.split(',') if x.strip()]
        except Exception as e:
            self.get_logger().fatal("Invalid format for path_points; please supply comma-separated numbers.")
            self.raw = []
        if len(self.raw) % 2 != 0:
            self.get_logger().fatal('path_points must have an even number of elements (x,y pairs)')
        self.points = [[self.raw[i], self.raw[i+1]] for i in range(0, len(self.raw), 2)]

        if not self.points:
            self.get_logger().error('No path points specified!')

        # Register dynamic parameter callback once here
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.update_parameters()

        # Initialize index to -1 to wait for the first /next_goal message
        self.index = -1

        # publisher & subscriber
        self.goal_pub = self.create_publisher(Pose2D, 'goal', 10)
        self.create_subscription(Empty, 'next_goal', self._next_goal_cb, 10)

        self.get_logger().info("Path Gen. Initialized!")
        self.get_logger().debug(f'Path points: {self.points}')
        self.get_logger().debug("Waiting for /next_goal message to publish the first point.")

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
                self.get_logger().info(f'Updating path_points param to: {param.value}')
                try:
                    new_raw = [float(x.strip()) for x in param.value.split(',') if x.strip()]
                    if len(new_raw) % 2 != 0:
                        self.get_logger().fatal("path_points must have an even number of elements (x,y pairs).")
                        return SetParametersResult(successful=False)
                    self.raw = new_raw
                    self.points = [[self.raw[i], self.raw[i+1]] for i in range(0, len(self.raw), 2)]
                    self.get_logger().debug(f'Updated path points: {self.points}')
                except Exception as e:
                    self.get_logger().error("Error parsing path_points: " + str(e))
                    return SetParametersResult(successful=False)
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
