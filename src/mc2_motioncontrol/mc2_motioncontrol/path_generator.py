'''
Path Generator Node, publishes goal points to the /goal topic.
Recieves all goal points from user defined parameters.
The path is defined in a 2D coordinate system by Pose2D messages.
The path is defined by a list of x,y coordinates.
Theta in Pose2D is set to 0.0 by default.
Recieves trigger messages on /next_goal to publish the next point.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        # load parameters
        raw = self.declare_parameter('path_points', [0.0, 0.0]).value
        if len(raw) % 2 != 0:
            self.get_logger().fatal('path_points must have an even number of elements (x,y pairs)')
        self.points = [[raw[i], raw[i+1]] for i in range(0, len(raw), 2)]

        if not self.points:
            self.get_logger().error('No path points specified!')

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
