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
from ament_index_python.packages import get_package_share_directory

class DepthAnythingNode(Node):
    def __init__(self):
        super().__init__('depth_anything_node')
        
        # 1. Path Setup - Use your new exported model name
        pkg_share = get_package_share_directory('pyperception_pkg')
        model_path = os.path.join(pkg_share, 'models', 'depth_vits_308.onnx')
        cache_path = os.path.join(pkg_share, 'models', 'cache_depth')
        
        if not os.path.exists(cache_path):
            os.makedirs(cache_path)

        # 2. TensorRT Configuration for JetPack 6.2
        providers = [
            ('TensorrtExecutionProvider', {
                'device_id': 0,
                'trt_fp16_enable': True,
                'trt_engine_cache_enable': True,
                'trt_engine_cache_path': cache_path,
                'trt_builder_optimization_level': 3,
            }),
            'CUDAExecutionProvider',
            'CPUExecutionProvider'
        ]

        self.get_logger().info(f'Loading Depth Engine: {model_path}')
        # This will take a few minutes on the first run to build the TRT Engine
        self.session = ort.InferenceSession(model_path, providers=providers)
        
        self.input_name = self.session.get_inputs()[0].name
        # Match your export shape exactly
        self.imgsz = (308, 308) 

        # Normalization constants (ImageNet)
        self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(1, 1, 3)
        self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(1, 1, 3)

        # ROS Infrastructure
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.pub_depth = self.create_publisher(Image, '/depth/visual', 10)
        self.bridge = CvBridge()
        self.get_logger().info('Depth Anything V2 Node Ready')

    def preprocess(self, frame):
        # 1. Resize and convert to RGB
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, self.imgsz)
        
        # 2. Normalize: (x / 255.0 - mean) / std
        img = img.astype(np.float32) / 255.0
        img = (img - self.mean) / self.std
        
        # 3. HWC to NCHW
        img = img.transpose((2, 0, 1))[np.newaxis, :, :, :]
        return img

    def image_callback(self, msg):
        start_time = time.perf_counter()
        
        # Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]

        # 1. Preprocess
        img_input = self.preprocess(frame)

        # 2. Inference
        outputs = self.session.run(None, {self.input_name: img_input})
        depth = outputs[0].squeeze()

        # 3. Post-process (Normalization for visualization)
        # We use min-max normalization to fit the depth into 0-255
        depth_min = depth.min()
        depth_max = depth.max()
        
        if depth_max - depth_min > 1e-5:
            depth_norm = (depth - depth_min) / (depth_max - depth_min) * 255.0
        else:
            depth_norm = depth * 0.0
            
        depth_norm = depth_norm.astype(np.uint8)
        
        # Resize back to original camera resolution
        depth_resized = cv2.resize(depth_norm, (w, h))
        
        # Apply Magma colormap (Yellow is close, Purple/Black is far)
        depth_color = cv2.applyColorMap(depth_resized, cv2.COLORMAP_MAGMA)

        # 4. Performance Overlay
        fps = 1.0 / (time.perf_counter() - start_time)
        cv2.putText(depth_color, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 5. Publish
        self.pub_depth.publish(self.bridge.cv2_to_imgmsg(depth_color, 'bgr8'))

def main():
    rclpy.init()
    node = DepthAnythingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()