'''
Aux node for localization
Odometry node for robot localization using wheel encoders
This node subscribes to the left and right wheel encoder topics
and computes the robot's pose (x, y, theta) using the odometry equations.
It publishes the pose to the /pose topic.
'''

import rclpy 
import rclpy.logging
from rclpy.node import Node 
from geometry_msgs.msg import Pose2D 
from std_msgs.msg import Float32 
from rclpy import qos 
import numpy as np 
 
#This class will compute the pose of the robot from the encoder readings. 
# This node subscribes to the /VelocityEncR and /VelocityEncL topics 
# This node publishes the pose of the robot to the /pose topic.  

class Odometry(Node):  

    def __init__(self):  
        super().__init__('odometry_node') 
        ###########  INIT PUBLISHERS ################ 
        self.pub_pose = self.create_publisher(Pose2D, 'pose', 10)  
        ############## SUBSCRIBERS ##################  
        self.create_subscription(Float32, "VelocityEncR",  self.wr_cb, qos.qos_profile_sensor_data)  #qos_profile_sensor_data is a predefined QoS profile in ROS2 that is suitable for sensor data.
        # It is used to set the reliability and durability of the messages.
        self.create_subscription(Float32, "VelocityEncL",  self.wl_cb, qos.qos_profile_sensor_data)  
        ############ ROBOT CONSTANTS ################  
        self.r=0.05 #wheel radius for our simulated robot[m] 
        self.L=0.174 #wheel separation for our simulated robot [m] 
        self.wl = 0.0 #Left wheel speed [rad/s] 
        self.wr = 0.0 #Right wheel speed [rad/s] 
        self.x = 0.0 #Robot position in x-axis [m] 
        self.y = 0.0 #Robot position in y-axis [m] 
        self.theta = 0.0 #Robot orientation [rad] 

        #logger config
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG) # Set logger to DEBUG level
        self.get_logger().debug("Logger set to DEBUG level")
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG) # Set rclpy logger to DEBUG level

        self.robot_pose = Pose2D() 
        self.prev_time_ns = self.get_clock().now().nanoseconds  # Get the current time in nanoseconds

        timer_period = 0.05 
        self.create_timer(timer_period, self.main_timer_cb) 
        self.get_logger().info("Odometry Initialized!") 
     

    def main_timer_cb(self): 
        v,w = self.get_robot_velocity(self.wl, self.wr) #get the robot's speed from the encoders
        self.update_robot_pose(v, w) #update the robot's pose
        #print the robot's pose
        print("Robot pose: ", self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta)

        #publish the robot's pose
        self.pub_pose.publish(self.robot_pose)


    def wl_cb(self, wl):  
        ## This function receives the left wheel speed from the encoders  
        self.wl = wl.data

    def wr_cb(self, wr):  
        ## This function receives the right wheel speed from the encoders 
        self.wr = wr.data 
    
    def get_robot_velocity(self, wl, wr):
        ## This function computes the robot's velocity from the wheel speeds 

        # Compute the robot's velocity from the wheel speeds 
        v = self.r * (wr + wl) / 2.0  # Robot's linear velocity [m/s]
        w = self.r * (wr - wl) / self.L  # Robot's angular velocity [rad/s]
        #print ("v: ", v, "w: ", w)
        print("v: ", v, "w: ", w)
        return v, w
    
    def update_robot_pose(self, v, w):
        dt = (self.get_clock().now().nanoseconds - self.prev_time_ns) * 1e-9  # Convert to seconds
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += w * dt
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))  # Normalize the angle

        # Update the robot pose message to publish Pose2D
        self.robot_pose.x = self.x
        self.robot_pose.y = self.y
        self.robot_pose.theta = self.theta
        self.prev_time_ns = self.get_clock().now().nanoseconds  # Update the previous time


def main(args=None): 
    rclpy.init(args=args) 
    my_node=Odometry() 
    rclpy.spin(my_node) 
    my_node.destroy_node() 
    rclpy.shutdown() 

     
if __name__ == '__main__': 
    main()