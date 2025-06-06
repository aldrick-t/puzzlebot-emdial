import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('tlts_detector')
        self.bridge = CvBridge()

        # Create a named window for display
        self.window_name = 'Traffic Light Detection'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
        # Confidence threshold
        self.conf_threshold = 0.78

        # Subscribe to your camera feed
        self.create_subscription(
            Image,
            '/video_source/raw',
            self.image_callback,
            10
        )
        # Publisher for detected color
        self.publisher_tl = self.create_publisher(
            String,
            '/traffic_light',
            10
        )

        self.publisher_ts = self.create_publisher(
            String,
            '/traffic_sign',
            10
        )

        # Load your custom-trained YOLOv8 model
        self.model = YOLO('./src/puzzlebot_emdial/models/jun5_v8v8_e_uni_tlts.pt')

        # Map class indices → color strings
        self.class_map = {
            0: 'tl_green',
            1: 'tl_red',
            2: 'tl_yellow',
            3: 'ts_giveway',
            4: 'ts_left',
            5: 'ts_right',
            6: 'ts_stop',
            7: 'ts_straight',
            8: 'ts_work'
        }
        # BGR colors for drawing
        self.colors = {
            'tl_red':    (0,   0, 255),
            'tl_yellow': (0, 255, 255),
            'tl_green':  (0, 255,   0),
            'ts_giveway':(255,255,255),
            'ts_left':(100,255,255),
            'ts_right':(255,100,255),
            'ts_stop':(255,255,100),
            'ts_straight':(100,255,100),
            'ts_work':(100,100,255),
            'unknown':(255,255,255),
        }

    def image_callback(self, img_msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        results = self.model(frame)[0]

        # Default messages
        best_ts = ('none', 0.0)
        best_tl = ('none', 0.0)

        if results.boxes.shape[0] > 0.78:
            boxes     = results.boxes.xyxy.cpu().numpy()
            confs     = results.boxes.conf.cpu().numpy()
            class_ids = results.boxes.cls.cpu().numpy().astype(int)

            for box, conf, cls_id in zip(boxes, confs, class_ids):
                if conf < self.conf_threshold:
                    continue

                x1, y1, x2, y2 = map(int, box)
                class_name = self.class_map.get(cls_id, 'unknown')
                color = self.colors.get(class_name, self.colors['unknown'])

                # Draw bounding box and label
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                label = f"{class_name}: {conf:.2f}"
                cv2.putText(
                    frame, label, (x1, max(y1 - 10, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
                )

                # Track best tl_ and ts_ detection
                if class_name.startswith('ts_') and conf > best_ts[1]:
                    best_ts = (class_name, conf)
                elif class_name.startswith('tl_') and conf > best_tl[1]:
                    best_tl = (class_name, conf)
        else:
            cv2.putText(
                frame, 'No detection', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
            )

        # Show the frame with overlays
        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

        # Publish both messages
        ts_msg = String(data=best_ts[0])
        tl_msg = String(data=best_tl[0])
        self.publisher_ts.publish(ts_msg)
        self.publisher_tl.publish(tl_msg)
        #self.get_logger().info(f'Published TS: "{ts_msg.data}", prob "{best_ts[1]}" ')
        #self.get_logger().info(f'Published TL: "{tl_msg.data}", prob "{best_tl[1]}"')



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