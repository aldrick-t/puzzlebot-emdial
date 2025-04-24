#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Pose2D 
from geometry_msgs.msg import Twist 
import numpy as np 
import signal # To handle Ctrl+C 
import sys # To exit the program 

#This class will compute the robot speed to go to the goal 
#It will use the pose from the odometry node and the goal from the path generator 
#The robot will stop when it is close to the goal 
class GoToGoal(Node):  
    def __init__(self):  
        super().__init__('go_to_goal') 
        ###########  INIT PUBLISHERS ################ 
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10) 
        # Handle shutdown gracefully (stop the robot) 
        signal.signal(signal.SIGINT, self.shutdown_function) 
        ############## INIT SUBSCRIBERS ##################  
        self.create_subscription(Pose2D, "pose", self.pose_cb, 10)  
        self.create_subscription(Pose2D, "goal", self.goal_cb, 10) 
        ############ ROBOT CONSTANTS ################  
        self.goal_received = False 
        self.xg = 0.0 # Goal position x[m] 
        self.yg = 0.0 # Goal position y[m] 
 
        self.robot_pose = Pose2D() 
        self.xr = 0.0 # Robot position x[m] 
        self.yr = 0.0 # Robot position y[m] 
        self.theta_r = 0.0 # Robot orientation [rad] 
        self.theta_g = 0.0 # Goal orientation [rad]
        self.k_v = 0.5
        self.k_w = 1.0

        self.cmd_vel = Twist() 
        timer_period = 0.05  
        self.create_timer(timer_period, self.main_timer_cb) 
        self.get_logger().info("Node initialized!!") 


    def main_timer_cb(self): 
        ## This function is called every 0.05 seconds 
        if self.goal_received:
            print("Goal received") 
            print("Do something here") 
            ed, et = self.get_errors(self.xr, self.yr, self.xg, self.yg, self.theta_r)
            # Compute the robot's speed to go to the goal
            if ed < 0.1:
                print("Stop")
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.goal_received = False  # Reset the goal_received flag 
            else:  
                self.cmd_vel.linear.x = self.k_v * ed
                self.cmd_vel.angular.z = self.k_w * et
            self.pub_cmd_vel.publish(self.cmd_vel) 

        else:  
            print("Waiting for goal") 
            print("Publish the goal to the /goal topic") 

    def get_errors(self, xr, yr, xg, yg, theta_r):
        ## This function computes the errors in x and y 
        # Compute the errors in distance and angle
        ed = np.sqrt((xg - xr)**2 + (yg - yr)**2)
        theta_g = np.arctan2(yg - yr, xg - xr)  # Angle to the goal
        et = theta_g - theta_r  # Angle error
        et = np.arctan2(np.sin(et), np.cos(et))
        print("ed: ", ed, "et: ", et)
        return ed, et  # Return the errors in distance and angle
    
    def pose_cb(self, pose):  
        ## This function receives the /pose from the odometry_node 
        self.xr = pose.x 
        self.yr = pose.y 
        self.theta_r = pose.theta 
 
    def goal_cb(self, goal):  
        ## This function receives the /goal from the path_generator node 
        self.xg = goal.x 
        self.yg = goal.y 
        self.theta_g = goal.theta 
        self.goal_received = True 
        self.get_logger().info("Goal Received")  

    def shutdown_function(self, signum, frame): 
        self.get_logger().info("Shutting down. Stopping robot...") 
        stop_twist = Twist()  # All zeros to stop the robot 
        self.pub_cmd_vel.publish(stop_twist) 
        rclpy.shutdown() 
        sys.exit(0) 

def main(args=None): 
    rclpy.init(args=args) 
    my_node=GoToGoal() 
    rclpy.spin(my_node) 
    my_node.destroy_node() 
    rclpy.shutdown() 
     
if __name__ == '__main__': 

    main() 