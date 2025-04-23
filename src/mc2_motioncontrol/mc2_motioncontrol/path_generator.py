'''
Path Generator Node, publishes goal points to the /goal topic.
Recieves all goal points from user defined parameters.
The path is defined in a 2D coordinate system.
Recieves trigger messages on /next_point to publish the next point.
Decides control mode and gains.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        # load parameters: path: list of [x,y], control_modes: list of str, gains: list of [k_p,k_d]
        self.points = self.declare_parameter('path', [0, 0]).value
        self.control_modes = self.declare_parameter('control_modes', []).value
        self.gains = self.declare_parameter('gains', [0, 0]).value

        if not self.points:
            self.get_logger().error('No path points specified!')
        self.index = 0

        # publisher & subscriber
        self.goal_pub = self.create_publisher(Pose2D, 'goal', 10)
        self.create_subscription(Empty, 'next_point', self._next_point_cb, 10)

        # publish first point if available
        if self.points:
            self._publish(self.index)

    def _next_point_cb(self, msg):
        self.index += 1
        if self.index >= len(self.points):
            self.get_logger().info('Reached end of path, no more points.')
            return
        self._publish(self.index)

    def _publish(self, idx):
        point = self.points[idx]
        mode = self.control_modes[idx] if idx < len(self.control_modes) else 'default'
        gain = self.gains[idx] if idx < len(self.gains) else []

        msg = Pose2D()
        msg.x = float(point[0])
        msg.y = float(point[1])
        msg.theta = 0.0  # Default value for theta, update if needed

        self.goal_pub.publish(msg)

        self.get_logger().info(
            f'Publishing point #{idx}: {point} | mode: {mode} | gains: {gain}'
        )

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
