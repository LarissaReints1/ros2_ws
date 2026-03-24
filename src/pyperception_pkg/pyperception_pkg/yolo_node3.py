#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime as ort
import os
import time
from geometry_msgs.msg import PointStamped
from ament_index_python.packages import get_package_share_directory

class YoloOnnxNode(Node):
    def __init__(self):
        super().__init__('yolo_onnx_node')
        
        # 1. Locate model and setup cache directory
        pkg_share = get_package_share_directory('pyperception_pkg')
        model_path = os.path.join(pkg_share, 'models', 'yolov8n.onnx')
        
        # Create a cache folder so we don't have to rebuild the engine every time
        cache_path = os.path.join(pkg_share, 'models', 'cache')
        if not os.path.exists(cache_path):
            os.makedirs(cache_path)
            self.get_logger().info(f'Created TensorRT cache directory at {cache_path}')

        # 2. Configure Providers with TensorRT Caching and FP16
        # This speeds up the Jetson Nano significantly
        providers = [
            ('TensorrtExecutionProvider', {
                'device_id': 0,
                'trt_fp16_enable': True,
                'trt_engine_cache_enable': True,
                'trt_engine_cache_path': cache_path,
            }),
            'CUDAExecutionProvider',
            'CPUExecutionProvider'
        ]
        
        self.get_logger().info(f'Loading model: {model_path}')
        try:
            self.session = ort.InferenceSession(model_path, providers=providers)
        except Exception as e:
            self.get_logger().error(f'Failed to create ONNX session: {e}')
            return

        # Get metadata
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape 
        self.imgsz = (self.input_shape[2], self.input_shape[3])
        
        active_provider = self.session.get_providers()[0]
        self.get_logger().info(f'Inference is active on: {active_provider}')

        # ROS Setup
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.pub_image = self.create_publisher(Image, '/yolo/image', 10)
        self.pub_points = self.create_publisher(PointStamped, '/yolo/targets', 10)
        self.bridge = CvBridge()

    def preprocess(self, img):
        """Resizes image to model input size using letterboxing."""
        h, w = img.shape[:2]
        scale = min(self.imgsz[0] / h, self.imgsz[1] / w)
        nh, nw = int(h * scale), int(w * scale)
        img_resized = cv2.resize(img, (nw, nh))
        
        canvas = np.full((self.imgsz[0], self.imgsz[1], 3), 114, dtype=np.uint8)
        pad_w, pad_h = (self.imgsz[1]-nw)//2, (self.imgsz[0]-nh)//2
        canvas[pad_h:pad_h+nh, pad_w:pad_w+nw, :] = img_resized
        
        # Convert to float32, normalize, and change to NCHW format
        img_in = canvas.transpose((2, 0, 1))[np.newaxis, :, :, :].astype(np.float32) / 255.0
        return img_in, scale, pad_w, pad_h

    def image_callback(self, msg):
        start_time = time.perf_counter()
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # 1. Preprocess
        img_in, scale, pad_w, pad_h = self.preprocess(frame)

        # 2. Run Inference
        inf_start = time.perf_counter()
        outputs = self.session.run(None, {self.input_name: img_in})
        inf_duration = (time.perf_counter() - inf_start) * 1000

        # 3. Post-process (YOLOv8 output is [1, 84, 2100])
        output = np.squeeze(outputs[0]).T 
        boxes, confs, class_ids = [], [], []
        
        for row in output:
            scores = row[4:]
            max_score = np.max(scores)
            if max_score > 0.4: # Conf threshold
                class_id = np.argmax(scores)
                x, y, w, h = row[:4]
                
                # Undo padding and scaling
                x1 = int((x - w/2 - pad_w) / scale)
                y1 = int((y - h/2 - pad_h) / scale)
                
                boxes.append([x1, y1, int(w/scale), int(h/scale)])
                confs.append(float(max_score))
                class_ids.append(int(class_id))

        # 4. Non-Maximum Suppression
        indices = cv2.dnn.NMSBoxes(boxes, confs, 0.4, 0.5)
        
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                # Draw Box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"ID:{class_ids[i]} {confs[i]:.2f}", (x, y-5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Publish Target Point (Center of Box)
                point_msg = PointStamped()
                point_msg.header = msg.header
                point_msg.point.x, point_msg.point.y = float(x + w//2), float(y + h//2)
                self.pub_points.publish(point_msg)

        # 5. Performance Monitoring
        total_fps = 1.0 / (time.perf_counter() - start_time)
        cv2.putText(frame, f"Inf: {inf_duration:.1f}ms | FPS: {total_fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # 6. Publish Output
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

def main():
    rclpy.init()
    node = YoloOnnxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()