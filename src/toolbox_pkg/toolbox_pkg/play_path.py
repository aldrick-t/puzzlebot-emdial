import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import time
from pathlib import Path

class TrajectoryReplayer(Node):
    def __init__(self):
        super().__init__('trajectory_replayer')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.load_and_replay()

    def load_and_replay(self):
        filepath = Path(__file__).parent / 'trajectory.json'
        with open(filepath, 'r') as f:
            trajectory = json.load(f)

        start_time = time.time()
        for item in trajectory:
            if item['topic'] != 'cmd_vel':
                continue
            # Wait until it's time to replay the command
            delay = item['time'] - (time.time() - start_time)
            if delay > 0:
                time.sleep(delay)

            msg = Twist()
            msg.linear.x = item['data']['linear']
            msg.angular.z = item['data']['angular']
            self.cmd_pub.publish(msg)
            self.get_logger().info(f"Replaying cmd_vel: {msg.linear.x}, {msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryReplayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
