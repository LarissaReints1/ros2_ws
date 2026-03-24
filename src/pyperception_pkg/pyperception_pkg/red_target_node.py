#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedTargetNode(Node):
    def __init__(self):
        super().__init__('red_target_node')

        # Subscribers
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        # Publishers
        self.pub_image = self.create_publisher(Image, '/perception/image', 10)
        self.pub_pixel = self.create_publisher(PointStamped, '/perception/target_pixel', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/perception/pose', 10)

        self.bridge = CvBridge()
        self.depth = 1.0  # meters
        self.fx = self.fy = self.cx0 = self.cy0 = None
        self.got_info = False

        self.prev_u = None
        self.prev_v = None
        self.alpha = 0.5
        self.min_area = 150

        # Target timeout
        self.last_seen_time = None
        self.target_timeout = 2.0  # seconds to hold last seen target

        self.get_logger().info("Perception node started")

    # -------------------- Camera Info --------------------
    def camera_info_callback(self, msg: CameraInfo):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx0, self.cy0 = msg.k[2], msg.k[5]
        self.got_info = True

    # -------------------- Pixel → Camera --------------------
    def pixel_to_camera(self, u, v, depth):
        x = (u - self.cx0) * depth / self.fx
        y = depth
        z = (v - self.cy0) * depth / self.fy
        return x, y, z

    # -------------------- Publish Pose --------------------
    def publish_pose(self, u, v, depth, header):
        pose = PoseStamped()
        pose.header = header
        pose.header.frame_id = "camera_link"
        if np.isnan(u) or np.isnan(v):
            pose.pose.orientation.w = 1.0
        else:
            x, y, z = self.pixel_to_camera(u, v, depth)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, -z
            pose.pose.orientation.w = 1.0
        self.pub_pose.publish(pose)

    # -------------------- Publish No Target --------------------
    def publish_no_target(self, frame, header):
        msg = PointStamped()
        msg.header = header
        msg.point.x = msg.point.y = msg.point.z = -1.0
        self.pub_pixel.publish(msg)
        self.publish_pose(np.nan, np.nan, np.nan, header)
        self.prev_u = self.prev_v = None
        ros_img = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        ros_img.header = header
        self.pub_image.publish(ros_img)

    # -------------------- Draw Axes --------------------
    def draw_axes(self, frame):
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.circle(frame, (cx, cy), 10, (255, 255, 255), -1)  # center
        length = 50
        # X right (red)
        cv2.arrowedLine(frame, (cx, cy), (cx + length, cy), (0, 0, 255), 2)
        cv2.putText(frame, "X", (cx + length + 5, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)
        # Z up (blue)
        cv2.arrowedLine(frame, (cx, cy), (cx, cy - length), (255, 0, 0), 2)
        cv2.putText(frame, "Z", (cx - 10, cy - length - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0),2)
        # Y depth (green, diagonally down-right)
        cv2.arrowedLine(frame, (cx, cy), (cx + int(length*0.7), cy + int(length*0.7)), (0, 255, 0), 2)
        cv2.putText(frame, "Y", (cx + int(length*0.7) +5, cy + int(length*0.7) +5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2)

    # -------------------- Image Callback --------------------
    def image_callback(self, msg: Image):
        if not self.got_info:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Detect red
        mask = cv2.bitwise_or(
        cv2.inRange(hsv, (0, 150, 100), (5, 255, 255)),   
        cv2.inRange(hsv, (175, 150, 100), (180, 255, 255))  
    )
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.draw_axes(frame)

        valid = [c for c in contours if cv2.contourArea(c) > self.min_area]

        current_time = time.time()
        target_found = False

        if valid:
            # Target found
            largest = max(valid, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] != 0:
                u = int(M['m10']/M['m00'])
                v = int(M['m01']/M['m00'])
                # smoothing
                if self.prev_u is not None:
                    u = int(self.alpha*u + (1-self.alpha)*self.prev_u)
                    v = int(self.alpha*v + (1-self.alpha)*self.prev_v)
                self.prev_u, self.prev_v = u, v
                self.last_seen_time = current_time
                target_found = True

        # Check timeout: hold last target
        if not target_found:
            if self.prev_u is not None and self.last_seen_time is not None and \
               current_time - self.last_seen_time < self.target_timeout:
                u, v = self.prev_u, self.prev_v
                target_found = True
            else:
                u = v = -1
                self.prev_u = self.prev_v = None

        # Publish pixel
        point = PointStamped()
        point.header = msg.header
        point.point.x = float(u)
        point.point.y = float(v)
        point.point.z = float(self.depth if u != -1 else -1)
        self.pub_pixel.publish(point)

        # Publish pose
        self.publish_pose(u, v, self.depth if u != -1 else np.nan, msg.header)

        # Draw target if valid
        if target_found and valid:
            largest = max(valid, key=cv2.contourArea)
            cv2.drawContours(frame, [largest], -1, (0,255,0), 2)
            cv2.circle(frame, (u,v), 8, (0,0,255), 2)

        # Publish image
        ros_img = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        ros_img.header = msg.header
        self.pub_image.publish(ros_img)


def main(args=None):
    rclpy.init(args=args)
    node = RedTargetNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()