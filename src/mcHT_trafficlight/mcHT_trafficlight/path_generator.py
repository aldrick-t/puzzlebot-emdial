'''
Path Generator Node, publishes goal points to the /goal topic.
Recieves all goal points from user defined parameters.
The path is defined in a 2D coordinate system by Pose2D messages.
The path is defined by a list of x,y coordinates.
Theta in Pose2D is set to 0.0 by default.
Recieves trigger messages on /next_goal to publish the next point.
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

        # declare the parameter with a descriptor for dynamic reconfigure
        from rcl_interfaces.msg import ParameterDescriptor
        pts_descriptor = ParameterDescriptor(description="List of x,y coordinates for path points (even number of elements).")
        self.raw = self.declare_parameter('path_points', [0.0, 0.0], pts_descriptor).value

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

    def _next_goal_cb(self, msg):
        # Increment index only after publishing the current point
        self.update_parameters()
        if self.index + 1 >= len(self.points):
            self.get_logger().info('Reached end of path, no more points.')
        self.index += 1
        self._publish(self.index)

    def _publish(self, idx):
        point = self.points[idx]

        msg = Pose2D()
        msg.x = point[0]
        msg.y = point[1]
        msg.theta = 0.0  # Default value for theta, update if needed

        self.goal_pub.publish(msg)

        self.get_logger().info(f'Publishing point #{idx}: {point}')
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'path_points' and param.value != self.raw:
                self.get_logger().info(f'Updating path points to: {param.value}')
                self.raw = param.value
                if len(self.raw) % 2 != 0:
                    self.get_logger().fatal('path_points must have an even number of elements (x,y pairs)')
                self.points = [[self.raw[i], self.raw[i+1]] for i in range(0, len(self.raw), 2)]
                self.get_logger().debug(f'Updated path points: {self.points}')
        return SetParametersResult(successful=True)
    def update_parameters(self):
        # Static
        self.camera_topic = self.get_parameter('path_points').value
        


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
