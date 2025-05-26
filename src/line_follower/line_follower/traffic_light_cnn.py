'''
traffic_light_cnn.py
Traffic Light CNN Node
Node subscribes to camera image and publishes traffic light state.
It should use a pre-trained CNN model to classify traffic light states, that is on the models directory.
And the model was trained using pyTorch. So the extension of the model is .pth.
The model should be loaded using torch.load() and the image should be pre-processed using torchvision.transforms.

diegoquezadaco
MAY 2025

'''

import rclpy
from rclpy.node import Node
import torch
import torch.nn as nn
import torchvision.transforms as transforms
import torchvision.models as models
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.time import Time
import torch.nn.functional as F  # Required for softmax
from ultralytics import YOLO


class TrafficLightCNN(Node):
    '''
    Traffic Light CNN Node
    Subscriptions:
    - /camera/image_raw: Camera image
    Publishers:
    - /traffic_light_state: Traffic light state
    '''
    def __init__(self):
        super().__init__('traffic_light_cnn')
        self.bridge = CvBridge()
        
        self.wait_for_ros_time()
        
        # Logger initialization 
        # Set logging level default to DEBUG for all nodes
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.DEBUG)
        # Start logger
        self.logger = self.get_logger()
        
        #Init message
        self.logger.info("TrafficLightCNN Starting Initialization...")
        
        # Parameters immutable after startup sequence
        # None
        
        # Dynamic parameters
        # Logging level
        self.declare_parameter('log_severity', 'DEBUG')
        
        # Header vars
        self.img = None
        
        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        # Subscriptions
        # Image stream topic
        self.create_subscription(Image, '/video_source/raw', self.img_cb, 10)

        # Load the model
        self.model = models.resnet18(weights=None)

        num_classes = 3
        self.model.fc = nn.Linear(self.model.fc.in_features, num_classes)
        state_dict = torch.load('./src/line_follower/models/best_traffic_light_resnet.pth', map_location=torch.device('cpu'))
        self.model.load_state_dict(state_dict)
        self.model.eval()
        self.logger.info("Model loaded successfully.")
        # Define the image transformation
        #consider that the model was trained with images of size 120x120
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            transforms.Resize((120, 120))
        ])
        # Define the class labels
        self.class_labels = ['red', 'yellow', 'green']
        # Create a publisher for the traffic light state
        self.traffic_light_state_pub = self.create_publisher(String, '/traffic_light_state', 10)
        # QoS settings
        qos_profile = QoSProfile(depth=10)
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'log_severity':
                if param.value == 'DEBUG':
                    self.get_logger().set_level(LoggingSeverity.DEBUG)
                elif param.value == 'INFO':
                    self.get_logger().set_level(LoggingSeverity.INFO)
                elif param.value == 'WARN':
                    self.get_logger().set_level(LoggingSeverity.WARN)
                elif param.value == 'ERROR':
                    self.get_logger().set_level(LoggingSeverity.ERROR)
                elif param.value == 'FATAL':
                    self.get_logger().set_level(LoggingSeverity.FATAL)
                else:
                    self.get_logger().error("Invalid log severity level")
        return SetParametersResult(successful=True)
    def image_callback(self, msg):
        ''''
        Callback function for camera image topic
        consider that the model only 
        '''

    def img_cb(self, msg):
        '''
        Callback function for camera image topic
        '''
        # Convert the ROS image message to a CV2 image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # resize image to 120x120
        cv_image = cv2.resize(cv_image, (120, 120))
        # Convert the image to a tensor
        image_tensor = self.transform(cv_image)
        # Add a batch dimension
        image_tensor = image_tensor.unsqueeze(0)
        # Make a prediction
        with torch.no_grad():
            output = self.model(image_tensor)
            # Apply softmax to get probabilities
            probabilities = F.softmax(output, dim=1)

            # Get the predicted class index and its probability
            predicted_prob, predicted_index = torch.max(probabilities, 1)
            
            if predicted_prob.item() < 0.97:
                traffic_light_state = "off"
            else:
                traffic_light_state = self.class_labels[predicted_index.item()]
                
        # Create a String message
        state_msg = String()
        state_msg.data = traffic_light_state
        # Publish the traffic light state
        self.traffic_light_state_pub.publish(state_msg)
        # Log the traffic light state
        self.get_logger().info(f"Traffic light state: {traffic_light_state}")
    def wait_for_ros_time(self):
        '''
        Wait for the ROS time to be set
        '''
        while rclpy.ok():
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        #self.get_logger().info('ROS time is active!')
        self.get_logger().info("ROS time is set to current time.")


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightCNN()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
# This is a ROS2 node that uses a pre-trained CNN model to classify traffic light states.
# The node subscribes to a camera image topic and publishes the traffic light state.