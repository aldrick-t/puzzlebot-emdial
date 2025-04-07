# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

#Class Definition
class MyNode(Node):
    def __init__(self):
        super().__init__('my_publisher')
        #ros publisher to /signal topic
        self.signal_pub = self.create_publisher(Float32, 'signal', 10)

        #ros timer
        self.timer1 = self.create_timer(1.0, self.timer1_cb)
        print("node started, publishing signal...")

    #Timer Callback
    def timer1_cb(self):
        print("In timer Callback")
        #create ros msg
        signal_msg = Float32()
        #populate msg
        signal_msg.data = 77.0
        #send/publish msg
        self.signal_pub.publish(signal_msg)

#Main
def main(args=None):
    rclpy.init(args=args)

    my_node  = MyNode()

    rclpy.spin(my_node) #inf loop to keep node alive

    rclpy.shutdown() #close ros comms

    my_node.destroy_node() #mem cleanup


#Execute Node
if __name__ == '__main__':
    main()