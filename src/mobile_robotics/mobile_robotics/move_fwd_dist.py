import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32

class MoveForwardClass(Node):
    def __init__(self):
        super().__init__('move_linear') #Init the node with the name "move_linear"
        # Declare necessary variables
        self.start_time = self.get_clock().now() #Indicate the time when the robot starts moving. 
        self.stop_time = 10.0 #Stop after some seconds
        self.state = "stop" #The robot will be initially stopped
        self.first_time = True

        self.d = 0.0 # Distance to move [m]
        self.v = 0.2 # Linear speed [m/s]
        self.t = 0.0 # Time to move requested distance [s]
        self.distance_received = False #Flag to indicate that the distance has been received


        # Init ROS subscriber
        self.dist_sub = self.create_subscription(Float32, "distance", self.distance_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # Declare the ROS timer
        timer_period = 0.05 #Time in seconds to call the timer_callback function
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Node initialized!!!")
        self.vel = Twist()
    
    def distance_callback(self, dist_msg):
        self.d = dist_msg.data #Distance to move [m]
        self.t = self.d / self.v #Time to move [s]
        self.distance_received = True #Flag to indicate that the distance has been received

    def timer_callback(self):
        if self.state == "stop":
            self.vel.linear.x = 0.0 # m/s
            self.vel.angular.z = 0.0 # rad/s
            self.cmd_vel_pub.publish(self.vel) #publish the message
            if self.distance_received:
                self.distance_received = False
                self.state = "move_linear" #Change the state to move forward
                self.start_time = self.get_clock().now() #Update the time when the robot started moving
                self.get_logger().info("Moving forward")
    
        elif self.state == "move_linear":
            self.vel.linear.x = self.v # m/s
            self.vel.angular.z = 0.0 # rad/s
            self.cmd_vel_pub.publish(self.vel) #publish the message
            if self.get_clock().now().nanoseconds - self.start_time.nanoseconds >= int(self.t*10**9):
                self.state = "stop" #Change the state to stop
                self.get_logger().info("Stopping")
        

def main(args=None):
    rclpy.init(args=args)
    m_p=MoveForwardClass()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
