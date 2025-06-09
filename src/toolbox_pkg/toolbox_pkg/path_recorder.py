import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import json
import time
from pathlib import Path

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.pose_sub = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.trajectory = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9  # seconds

    def cmd_callback(self, msg):
        self.record('cmd_vel', msg)

    def pose_callback(self, msg):
        self.record('pose', msg)

    def record(self, topic, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        timestamp = current_time - self.start_time
        if topic == 'cmd_vel':
            data = {
                'linear': msg.linear.x,
                'angular': msg.angular.z
            }
        else:  # 'pose'
            data = {
                'position': {
                    'x': msg.position.x,
                    'y': msg.position.y,
                    'z': msg.position.z
                },
                'orientation': {
                    'x': msg.orientation.x,
                    'y': msg.orientation.y,
                    'z': msg.orientation.z,
                    'w': msg.orientation.w
                }
            }
        self.trajectory.append({
            'time': timestamp,
            'topic': topic,
            'data': data
        })

    def save_to_file(self):
        filepath = Path(__file__).parent / 'trajectory.json'
        with open(filepath, 'w') as f:
            json.dump(self.trajectory, f, indent=2)
        self.get_logger().info(f"Trajectory saved to {filepath}")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_to_file()
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
