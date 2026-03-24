#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorrt as trt
from geometry_msgs.msg import PointStamped
import os
from ament_index_python.packages import get_package_share_directory


class YoloTRTNode(Node):
    def __init__(self):
        super().__init__('yolo_node2')

        # Subscribers
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)

        # Publishers
        self.pub_image = self.create_publisher(Image, '/yolo/image', 10)
        self.pub_points = self.create_publisher(PointStamped, '/yolo/targets', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # TensorRT Logger and Engine
        self.logger = trt.Logger(trt.Logger.INFO)
        pkg_path = get_package_share_directory('pyperception_pkg')
        self.engine_file = os.path.join(pkg_path, "models", "yolov8n.engine")
        self.get_logger().info(f'Loading TensorRT engine from {self.engine_file}...')
        self.engine = self.load_engine(self.engine_file)
        self._context = self.engine.create_execution_context()
        self.get_logger().info('TensorRT engine loaded successfully.')

        # Allocate buffers
        self.inputs, self.outputs, self.bindings = self.allocate_buffers()

        # Model input size (match engine)
        self.input_h = 320
        self.input_w = 320

    def load_engine(self, engine_file_path):
        with open(engine_file_path, "rb") as f:
            runtime = trt.Runtime(self.logger)
            engine = runtime.deserialize_cuda_engine(f.read())
            if engine is None:
                self.get_logger().error(f"Failed to load TensorRT engine: {engine_file_path}")
            return engine

    def allocate_buffers(self):
        inputs, outputs, bindings = [], [], []

        for i in range(self.engine.num_io_tensors):  # use num_io_tensors
            name = self.engine.get_tensor_name(i)
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            shape = self.engine.get_tensor_shape(name)
            size = trt.volume(shape)

            host_mem = np.empty(size, dtype=dtype)
            bindings.append(int(host_mem.ctypes.data))

            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                inputs.append(host_mem)
            else:
                outputs.append(host_mem)

        return inputs, outputs, bindings

    def letterbox(self, img, new_shape=(320, 320), color=(114, 114, 114)):
        """Resize image with unchanged aspect ratio using padding (letterbox)."""
        h, w = img.shape[:2]
        scale = min(new_shape[0]/h, new_shape[1]/w)
        nh, nw = int(h*scale), int(w*scale)
        img_resized = cv2.resize(img, (nw, nh))
        canvas = np.full((new_shape[0], new_shape[1], 3), color, dtype=np.uint8)
        top = (new_shape[0]-nh)//2
        left = (new_shape[1]-nw)//2
        canvas[top:top+nh, left:left+nw, :] = img_resized
        return canvas, scale, top, left

    def preprocess_image(self, frame):
        img, _, _, _ = self.letterbox(frame, new_shape=(self.input_w, self.input_h))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC → CHW
        img = np.expand_dims(img, axis=0)
        return img

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img, scale, top, left = self.letterbox(frame, new_shape=(self.input_w, self.input_h))

        # Preprocess
        img, scale, top, left = self.letterbox(frame, new_shape=(320, 320))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0       # use float32
        img = np.transpose(img, (2, 0, 1))         # HWC → CHW
        img = np.expand_dims(img, axis=0)          # add batch dim
        img = np.ascontiguousarray(img)            # ensure contiguous memory
        np.copyto(self.inputs[0], img.ravel())

        # Run inference
        self._context.execute_v2(bindings=self.bindings)

        # Get output
        predictions = self.outputs[0]
        boxes = self.postprocess(predictions, frame.shape, scale, top, left)

        # Draw and publish
        for box in boxes:
            x1, y1, x2, y2, conf, cls = box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{int(cls)}:{conf:.2f}', (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Publish center point
            u = (x1 + x2)//2
            v = (y1 + y2)//2
            point_msg = PointStamped()
            point_msg.header = msg.header
            point_msg.point.x = float(u)
            point_msg.point.y = float(v)
            point_msg.point.z = 0.0
            self.pub_points.publish(point_msg)

        out_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out_msg.header = msg.header
        self.pub_image.publish(out_msg)

    def postprocess(self, preds, orig_shape, scale, top, left):
        """
        Convert flattened output to boxes/conf/class and map back to original frame.
        """
        num = preds.size // 6
        preds = preds[:num*6].reshape(-1,6)
        boxes = []
        for p in preds:
            x1, y1, x2, y2, conf, cls = p

            # Undo letterbox scaling
            x1 = (x1 - left) / scale
            x2 = (x2 - left) / scale
            y1 = (y1 - top) / scale
            y2 = (y2 - top) / scale

            # Clip to image size
            x1 = max(0, min(orig_shape[1]-1, int(x1)))
            x2 = max(0, min(orig_shape[1]-1, int(x2)))
            y1 = max(0, min(orig_shape[0]-1, int(y1)))
            y2 = max(0, min(orig_shape[0]-1, int(y2)))

            boxes.append([x1, y1, x2, y2, float(conf), int(cls)])
        return boxes


def main(args=None):
    rclpy.init(args=args)
    node = YoloTRTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()