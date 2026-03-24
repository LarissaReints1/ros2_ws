#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from geometry_msgs.msg import PointStamped

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # Subscribers
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)

        # Publishers
        self.pub_image = self.create_publisher(Image, '/yolo/image', 10)
        self.pub_points = self.create_publisher(PointStamped, '/yolo/targets', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # YOLO Model
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Loading YOLOv8 on {self.device}...')
        from ultralytics import YOLO
        self.model = YOLO('yolov8n.pt')  # Nano model
        self.model.to(self.device)
        self.get_logger().info('YOLOv8 loaded successfully.')

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # YOLO inference
        results = self.model(frame, device=self.device, verbose=False)[0]

        # Process detections
        if results.boxes:
            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls = int(box.cls[0])

                # Print what it sees
                print(f"Detected class {cls} with confidence {conf:.2f} at ({x1},{y1},{x2},{y2})")

                # Draw rectangle on image
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(frame, f'{cls}:{conf:.2f}', (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                # Publish center point
                u = (x1 + x2)//2
                v = (y1 + y2)//2
                point_msg = PointStamped()
                point_msg.header = msg.header
                point_msg.point.x = float(u)
                point_msg.point.y = float(v)
                point_msg.point.z = 0.0
                self.pub_points.publish(point_msg)

        # Publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out_msg.header = msg.header
        self.pub_image.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()