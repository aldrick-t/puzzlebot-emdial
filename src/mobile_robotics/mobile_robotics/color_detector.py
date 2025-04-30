""" This program publishes the radius and center of the detected ball   
    The radius will be zero if there is no detected object  
    published topics:  
        /processed_img [Image] 
    subscribed topics:  
        /camera    [Image]  
"""  

import rclpy 
from rclpy.node import Node 
import cv2 
import numpy as np 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
  
class CVExample(Node): 
    def __init__(self): 
        super().__init__('color_detector') 
        self.bridge = CvBridge() 

        self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10) 
        self.pub = self.create_publisher(Image, 'processed_img', 10) 
         
        self.image_received_flag = False #This flag is to ensure we received at least one image  
        dt = 0.1 
        self.timer = self.create_timer(dt, self.timer_callback) 
        self.get_logger().info('ros_color_tracker Node started') 

    def camera_callback(self, msg): 
        try:  
            # We select bgr8 because its the OpenCV encoding by default  
            self.cv_img= self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            self.image_received_flag = True  
        except: 
            self.get_logger().info('Failed to get an image') 

    def timer_callback(self): 
        if self.image_received_flag: 
            self.image_received_flag=False 
            # Resize the image to 160x120 
            resized_image = cv2.resize(self.cv_img, (160,120)) #(width, height) 
            # Add some text to the image 
            cv2.putText(resized_image, "Hello", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2) 
            self.pub.publish(self.bridge.cv2_to_imgmsg(resized_image,'bgr8')) 
 

def main(args=None): 
    rclpy.init(args=args) 
    cv_e = CVExample() 
    rclpy.spin(cv_e) 
    cv_e.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 