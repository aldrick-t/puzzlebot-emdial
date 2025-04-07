import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

class subsriberNode(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.signal_sub = self.create_subscription(Float32, 'signal', self.listener_callback, 10)
        self.processed_pub = self.create_publisher(Float32, 'processed_signal', 10)
        self.i = 0
    

    def listener_callback(self, my_msg):
        print(my_msg)

        processed_signal = Float32()
        processed_signal.data = my_msg.data + self.i

        self.processed_pub.publish(processed_signal)
        self.i += 1.0

#Main
def main(args=None):
    rclpy.init(args=args)

    subsriber  = subsriberNode()

    rclpy.spin(subsriber)

    rclpy.shutdown()

    subsriber.destroy_node()


#Execute Node
if __name__ == '__main__':
    main()