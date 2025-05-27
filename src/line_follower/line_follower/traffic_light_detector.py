import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        self.bridge = CvBridge()

        # Create a named window for display
        self.window_name = 'Traffic Light Detection'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # Subscribe to your camera feed
        self.create_subscription(
            Image,
            '/video_source/raw',
            self.image_callback,
            10
        )
        # Publisher for detected color
        self.publisher = self.create_publisher(
            String,
            '/traffic_light_color',
            10
        )

        # Load your custom-trained YOLOv8 model
        self.model = YOLO('./src/line_follower/models/best.pt')

        # Map class indices → color strings
        self.class_map = {
            0: 'green',
            1: 'red',
            2: 'yellow'
        }
        # BGR colors for drawing
        self.colors = {
            'red':    (0,   0, 255),
            'yellow': (0, 255, 255),
            'green':  (0, 255,   0),
            'unknown':(255,255,255)
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

        # Overlay detections
        if results.boxes.shape[0] > 0:
            # Extract numpy arrays
            boxes    = results.boxes.xyxy.cpu().numpy()
            confs    = results.boxes.conf.cpu().numpy()
            class_ids= results.boxes.cls.cpu().numpy().astype(int)

            for box, conf, cls_id in zip(boxes, confs, class_ids):
                x1, y1, x2, y2 = map(int, box)
                class_name    = self.class_map.get(cls_id, 'unknown')
                color         = self.colors.get(class_name, self.colors['unknown'])
                # Draw bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                # Draw label + confidence
                label = f"{class_name}: {conf:.2f}"
                cv2.putText(
                    frame, label, (x1, max(y1 - 10, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
                )

            # Determine top confidence for publishing
            best_idx       = int(confs.argmax())
            detected_color = self.class_map.get(class_ids[best_idx], 'unknown')
        else:
            # No detections: annotate and publish "none"
            cv2.putText(
                frame, 'No detection', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
            )
            detected_color = 'none'

        # Show the frame with overlays
        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)  # needed to refresh the window

        # Publish the detected traffic-light color
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
        cv2.destroyAllWindows()  # clean up the display window

if __name__ == '__main__':
    main()