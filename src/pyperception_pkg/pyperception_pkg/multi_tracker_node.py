import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial import distance as dist

class MultiTargetFusion(Node):
    def __init__(self):
        super().__init__('multi_target_fusion')
        self.bridge = CvBridge()
        
        # --- CONFIGURATION ---
        self.alpha = 0.4
        self.min_area = 1000
        self.max_area = 100000      
        self.max_disappeared = 25   
        
        # Initial placeholder intrinsics (will be overwritten by CameraInfo)
        self.fx = 500.0
        self.fy = 500.0
        self.cx = 320.0
        self.cy = 240.0
        self.intrinsics_ready = False

        self.next_id = 0
        self.objects = {}     
        self.rects = {}       
        self.disappeared = {} 
        self.current_depth = None
        
        # Subscriptions
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.create_subscription(Image, '/depth/visual', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        
        self.get_logger().info("3D Fusion Node waiting for CameraInfo...")

    def info_callback(self, msg):
        """Extracts intrinsic parameters from the ROS 2 CameraInfo message."""
        # P is the 3x4 projection matrix
        # [fx  0  cx  Tx]
        # [ 0  fy cy  Ty]
        # [ 0  0   1   0]
        self.fx = msg.p[0]
        self.fy = msg.p[5]
        self.cx = msg.p[2]
        self.cy = msg.p[6]
        
        if not self.intrinsics_ready:
            self.get_logger().info(f"Intrinsics Loaded: fx={self.fx:.2f}, cx={self.cx:.2f}")
            self.intrinsics_ready = True

    def depth_to_meters(self, intensity):
        """Maps 0-255 intensity to meters. Adjust based on your Depth Anything V2 scaling."""
        meters = 10.0 - (intensity / 255.0) * 9.5
        return max(0.1, meters)

    def depth_callback(self, msg):
        self.current_depth = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def image_callback(self, msg):
        if not self.intrinsics_ready:
            return # Wait for camera info before processing

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]

        # 1. Red Masking
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 150, 70), (10, 255, 255)) + \
               cv2.inRange(hsv, (170, 150, 70), (180, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        # 2. Detect Blobs
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        input_centroids, input_rects = [], []
        
        for c in contours:
            if self.min_area < cv2.contourArea(c) < self.max_area:
                (bx, by, bw, bh) = cv2.boundingRect(c)
                input_centroids.append((bx + bw//2, by + bh//2))
                input_rects.append((bx, by, bw, bh))

        # 3. Tracking
        self.update_tracker(input_centroids, input_rects)

        # 4. 3D Fusion
        for obj_id in list(self.objects.keys()):
            u, v = self.objects[obj_id]
            rx, ry, rw, rh = self.rects[obj_id]
            is_lost = self.disappeared[obj_id] > 0
            color = (0, 165, 255) if is_lost else (0, 255, 0)

            if self.current_depth is not None:
                dh, dw = self.current_depth.shape[:2]
                scx, scy = int(np.clip(u * dw / w, 0, dw-1)), int(np.clip(v * dh / h, 0, dh-1))
                
                z_m = self.depth_to_meters(self.current_depth[scy, scx][0])
                
                # --- 3D CALCULATION USING DYNAMIC INTRINSICS ---
                x_m = (u - self.cx) * z_m / self.fx
                y_m = (v - self.cy) * z_m / self.fy

                label = f"ID:{obj_id} [{x_m:.1f}, {y_m:.1f}, {z_m:.1f}m]"
                cv2.rectangle(frame, (rx, ry), (rx+rw, ry+rh), color, 2)
                cv2.putText(frame, label, (rx, ry-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow("3D Fusion (Dynamic Info)", frame)
        cv2.waitKey(1)

    def update_tracker(self, new_centroids, new_rects):
        if not new_centroids:
            for obj_id in list(self.disappeared.keys()):
                self.disappeared[obj_id] += 1
                if self.disappeared[obj_id] > self.max_disappeared:
                    self.deregister(obj_id)
            return

        if not self.objects:
            for i in range(len(new_centroids)):
                self.register(new_centroids[i], new_rects[i])
        else:
            object_ids = list(self.objects.keys())
            D = dist.cdist(np.array(list(self.objects.values())), np.array(new_centroids))
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]
            used_rows, used_cols = set(), set()

            for (row, col) in zip(rows, cols):
                if row in used_rows or col in used_cols: continue
                obj_id = object_ids[row]
                self.objects[obj_id] = tuple(((self.alpha * np.array(new_centroids[col])) + ((1-self.alpha) * np.array(self.objects[obj_id]))).astype(int))
                self.rects[obj_id] = tuple(((self.alpha * np.array(new_rects[col])) + ((1-self.alpha) * np.array(self.rects[obj_id]))).astype(int))
                self.disappeared[obj_id] = 0
                used_rows.add(row)
                used_cols.add(col)

            for i in range(len(new_centroids)):
                if i not in used_cols: self.register(new_centroids[i], new_rects[i])
            for i, o_id in enumerate(object_ids):
                if i not in used_rows:
                    self.disappeared[o_id] += 1
                    if self.disappeared[o_id] > self.max_disappeared: self.deregister(o_id)

    def register(self, centroid, rect):
        self.objects[self.next_id], self.rects[self.next_id], self.disappeared[self.next_id] = centroid, rect, 0
        self.next_id += 1

    def deregister(self, obj_id):
        del self.objects[obj_id], self.rects[obj_id], self.disappeared[obj_id]

def main():
    rclpy.init()
    rclpy.spin(MultiTargetFusion())
    rclpy.shutdown()