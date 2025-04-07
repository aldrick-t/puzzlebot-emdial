import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 

class MoveForwardClass(Node):
    def __init__(self):
        super().__init__('move_forward') #Init the node with the name "move_forward"
        # Declare necessary variables
        self.start_time = self.get_clock().now() #Indicate the time when the robot starts moving. 
        self.stop_time = 10.0 #Stop after some seconds
        self.state = "stop" #The robot will be initially stopped
        self.first_time = True
        # Init ROS subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # Declare the ROS timer
        timer_period = 0.05 #Time in seconds to call the timer_callback function
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Node initialized!!!")
        self.vel = Twist()
    
    def timer_callback(self):
        if self.state == "stop":
            self.vel.linear.x = 0.0 # m/s
            self.vel.angular.z = 0.0 # rad/s
            self.cmd_vel_pub.publish(self.vel) #publish the message
            if self.first_time:
                self.first_time = False
                self.state = "move_forward" #Change the state to move forward
                self.start_time = self.get_clock().now() #Update the time when the robot started moving
                self.get_logger().info("Moving forward")
    
        elif self.state == "move_forward":
            self.vel.linear.x = 0.2 # m/s
            self.vel.angular.z = 0.0 # rad/s
            self.cmd_vel_pub.publish(self.vel) #publish the message
            if self.get_clock().now().nanoseconds - self.start_time.nanoseconds >= self.stop_time*10**9:
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
