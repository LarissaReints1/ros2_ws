#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torch.nn.functional as F
import torch.hub

class RedTargetDepthNode(Node):
    def __init__(self):
        super().__init__('red_target_depth_node')

        # ---------------- Subscribers ----------------
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        # ---------------- Publishers ----------------
        self.pub_image = self.create_publisher(Image, '/perception/image', 10)
        self.pub_pixel = self.create_publisher(PointStamped, '/perception/target_pixel', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/perception/pose', 10)
        self.pub_depth = self.create_publisher(Image, '/perception/depth', 10)

        self.bridge = CvBridge()

        # ---------------- Camera intrinsics ----------------
        self.fx = self.fy = self.cx0 = self.cy0 = None
        self.got_info = False

        # ---------------- Target tracking ----------------
        self.prev_u = None
        self.prev_v = None
        self.prev_depth = None
        self.alpha = 0.5
        self.depth_alpha = 0.3
        self.min_area = 150
        self.last_seen_time = None
        self.target_timeout = 2.0

        # ---------------- Depth model (MiDaS) ----------------
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f"Loading MiDaS on {self.device}...")

        # Load MiDaS small model
        self.midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
        self.midas.to(self.device)
        self.midas.eval()

        # Load transforms (we will call them per frame)
        self.midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")

        # ---------------- Depth control ----------------
        self.latest_frame = None
        self.depth_map = None
        self.depth_timer = self.create_timer(0.2, self.depth_loop)  # 5 Hz

        self.get_logger().info("MiDaS loaded successfully.")

    # ---------------- Camera Info ----------------
    def camera_info_callback(self, msg: CameraInfo):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx0, self.cy0 = msg.k[2], msg.k[5]
        self.got_info = True

    # ---------------- Depth Loop ----------------
    # ---------------- Depth Loop ----------------
    def depth_loop(self):
        if self.latest_frame is None:
            return

        frame = self.latest_frame
        h, w = frame.shape[:2]

        # Preprocess for MiDaS
        transform = self.midas_transforms.small_transform()  # get the transform object
        input_batch = transform(frame)  # pass the frame here
        input_batch = input_batch.to(self.device)

        # Inference
        with torch.no_grad():
            prediction = self.midas(input_batch)
            prediction = F.interpolate(
                prediction.unsqueeze(1),
                size=(h, w),
                mode="bicubic",
                align_corners=False
            ).squeeze().cpu().numpy()

        depth = prediction if prediction is not None else np.nan
        self.depth_map = depth

        # Publish depth visualization
        if depth is not None and np.any(~np.isnan(depth)):
            depth_vis = (np.clip(depth, 0, np.percentile(depth, 99)) / np.max(depth) * 255).astype(np.uint8)
        else:
            depth_vis = np.zeros((h, w), dtype=np.uint8)

        depth_msg = self.bridge.cv2_to_imgmsg(depth_vis, encoding='mono8')
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = "camera_link"
        self.pub_depth.publish(depth_msg)

    # ---------------- Pixel → Camera ----------------
    def pixel_to_camera(self, u, v, depth):
        x = (u - self.cx0) * depth / self.fx
        y = depth
        z = (v - self.cy0) * depth / self.fy
        return x, y, z

    # ---------------- Publish Pose ----------------
    def publish_pose(self, u, v, depth, header):
        pose = PoseStamped()
        pose.header = header
        pose.header.frame_id = "camera_link"

        if np.isnan(depth):
            pose.pose.orientation.w = 1.0
        else:
            x, y, z = self.pixel_to_camera(u, v, depth)
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = -z
            pose.pose.orientation.w = 1.0

        self.pub_pose.publish(pose)

    # ---------------- Image Callback ----------------
    def image_callback(self, msg: Image):
        if not self.got_info:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.latest_frame = frame

        # Red color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, (0, 120, 70), (10, 255, 255)),
            cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
        )

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) > self.min_area]

        current_time = time.time()
        target_found = False

        if valid:
            largest = max(valid, key=cv2.contourArea)
            M = cv2.moments(largest)

            if M['m00'] != 0:
                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])

                if self.prev_u is not None:
                    u = int(self.alpha * u + (1 - self.alpha) * self.prev_u)
                    v = int(self.alpha * v + (1 - self.alpha) * self.prev_v)

                self.prev_u, self.prev_v = u, v

                if self.depth_map is not None:
                    depth = self.depth_map[v, u]
                    if self.prev_depth is not None:
                        depth = self.depth_alpha * depth + (1 - self.depth_alpha) * self.prev_depth
                    self.prev_depth = depth
                else:
                    depth = np.nan

                self.last_seen_time = current_time
                target_found = True

        if not target_found:
            if self.prev_u is not None and self.last_seen_time is not None and \
               current_time - self.last_seen_time < self.target_timeout:
                u, v = self.prev_u, self.prev_v
                depth = self.prev_depth
                target_found = True
            else:
                u = v = -1
                depth = np.nan
                self.prev_u = self.prev_v = self.prev_depth = None

        # Publish pixel and pose
        point = PointStamped()
        point.header = msg.header
        point.point.x = float(u)
        point.point.y = float(v)
        if depth is None or np.isnan(depth):
            point.point.z = -1.0
        else:
            point.point.z = float(depth)
        self.pub_pixel.publish(point)
        self.publish_pose(u, v, depth, msg.header)

        # Visualization
        if target_found and u != -1:
            cv2.circle(frame, (u, v), 8, (0, 0, 255), 2)

        ros_img = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        ros_img.header = msg.header
        self.pub_image.publish(ros_img)

def main(args=None):
    rclpy.init(args=args)
    node = RedTargetDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()