#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        self.bridge = CvBridge()
        # Subscribe to your camera feed
        self.create_subscription(
            Image,
            '/video_source/raw',
            self.image_callback,
            10
        )
        # Publish detected color
        self.publisher = self.create_publisher(
            String,
            '/traffic_light_color',
            10
        )
        # Load your custom-trained YOLOv8 model
        self.model = YOLO('./src/line_follower/models/best.pt')
        # Map class indices → color strings
        self.class_map = {
            0: 'red',
            1: 'yellow',
            2: 'green'
        }

    def image_callback(self, img_msg: Image):
        # Convert ROS Image → OpenCV image
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Run YOLO inference
        results = self.model(frame)[0]

        # No boxes → publish “no_detection”
        if results.boxes.shape[0] == 0:
            detected_color = 'no_detection'
        else:
            # Find the detection with highest confidence
            confs = results.boxes.conf.cpu().numpy()
            best_idx = int(confs.argmax())
            cls_id = int(results.boxes.cls[best_idx].cpu().numpy())
            detected_color = self.class_map.get(cls_id, 'unknown')

        # Publish result
        msg = String(data=detected_color)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published traffic light color: "{detected_color}"')

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()