import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from sensor_msgs.msg import JointState, CameraInfo
from std_msgs.msg import Float32
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np

class ServoTargetingNode(Node):
    def __init__(self):
        super().__init__('servo_targeting_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('pan_home_deg', 0.0)
        self.declare_parameter('tilt_home_deg', 0.0)
        self.declare_parameter('pan_min_deg', -90.0)
        self.declare_parameter('pan_max_deg', 90.0)
        self.declare_parameter('tilt_min_deg', -90.0)
        self.declare_parameter('tilt_max_deg', 30.0)
        self.declare_parameter('pan_offset', 90)
        self.declare_parameter('tilt_offset', 90)
        self.declare_parameter('use_vector_only', True)

        self.pan_home = self.get_parameter('pan_home_deg').value
        self.tilt_home = self.get_parameter('tilt_home_deg').value
        self.pan_min = self.get_parameter('pan_min_deg').value
        self.pan_max = self.get_parameter('pan_max_deg').value
        self.tilt_min = self.get_parameter('tilt_min_deg').value
        self.tilt_max = self.get_parameter('tilt_max_deg').value
        self.pan_offset = self.get_parameter('pan_offset').value
        self.tilt_offset = self.get_parameter('tilt_offset').value
        self.use_vector_only = self.get_parameter('use_vector_only').value

        self.desired_pan = self.pan_home
        self.desired_tilt = self.tilt_home

        # ---------------- TF2 ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- Camera calibration ----------------
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # ---------------- Subscribers ----------------
        self.sub_target_pixel = self.create_subscription(
            PointStamped,
            '/perception/target_pixel',
            self.target_pixel_callback,
            10
        )
        self.sub_target_pose = self.create_subscription(
            PoseStamped,
            '/perception/pose',
            self.target_pose_callback,
            10
        )
        self.sub_camera_info = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # ---------------- Publishers ----------------
        self.joint_pub = self.create_publisher(JointState, '/target/joint_states', 10)
        self.pub_servo1 = self.create_publisher(Float32, '/servo1_angle', 10)
        self.pub_servo2 = self.create_publisher(Float32, '/servo2_angle', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/target_pose_ee_frame', 10)

        # ---------------- State ----------------
        self.target_pixel = None
        self.target_pose = None

        # ---------------- Timer / Control Loop ----------------
        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz
        self.get_logger().info("Servo Targeting Node started with vector-only mode: {}".format(self.use_vector_only))

    # ---------------- Callbacks ----------------
    def target_pixel_callback(self, msg: PointStamped):
        self.target_pixel = msg

    def target_pose_callback(self, msg: PoseStamped):
        self.target_pose = msg

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    # ---------------- Utilities ----------------
    @staticmethod
    def deg2rad(deg):
        return deg * math.pi / 180.0

    @staticmethod
    def rad2deg(rad):
        return rad * 180.0 / math.pi

    def compute_angles_from_point(self, x, y, z):
        distance = math.sqrt(x ** 2 + y ** 2)
        if distance < 1e-6:
            distance = 1e-6

        pan_deg = self.rad2deg(math.atan2(x, y))
        tilt_deg = self.rad2deg(math.atan2(z, distance))

        pan_deg = max(self.pan_min, min(self.pan_max, pan_deg))
        tilt_deg = max(self.tilt_min, min(self.tilt_max, tilt_deg))

        return pan_deg, tilt_deg

    def compute_angles_from_pixel(self, u, v):
        if self.fx is None or self.fy is None:
            self.get_logger().warn("Camera info not yet received")
            return self.pan_home, self.tilt_home

        # Unit vector in camera frame (depth-independent)
        vec_cam = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        vec_cam /= np.linalg.norm(vec_cam)

        # Since camera rotation = ee_base_frame, we can skip TF rotation
        vec_base = vec_cam

        # Compute pan/tilt
        pan_deg = self.rad2deg(math.atan2(vec_base[0], vec_base[2])) ## x,y
        tilt_deg = self.rad2deg(math.atan2(-vec_base[1], vec_base[2])) ## z, dist

        # Apply limits
        pan_deg = max(self.pan_min, min(self.pan_max, pan_deg))
        tilt_deg = max(self.tilt_min, min(self.tilt_max, tilt_deg))

        return pan_deg, tilt_deg

    def is_invalid(self, x, y, z):
        return any(math.isnan(v) or abs(v) > 1000.0 for v in (x, y, z))

    # ---------------- Control Loop ----------------
    def control_loop(self):
        go_home = False

        # Vector-only targeting (depth-independent)
        if self.use_vector_only and self.target_pixel is not None:
            u = self.target_pixel.point.x
            v = self.target_pixel.point.y
            self.desired_pan, self.desired_tilt = self.compute_angles_from_pixel(u, v)

        # 3D pose targeting (primary)
        elif self.target_pose is not None:
            if not self.target_pose.header.frame_id:
                self.get_logger().warn("Target pose has no frame_id")
                go_home = True
            else:
                try:
                    target_tf: PoseStamped = self.tf_buffer.transform(
                        self.target_pose,
                        'ee_base_frame',
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    self.target_pub.publish(target_tf)

                    x = target_tf.pose.position.x
                    y = target_tf.pose.position.y
                    z = target_tf.pose.position.z

                    if self.is_invalid(x, y, z):
                        go_home = True
                    else:
                        self.desired_pan, self.desired_tilt = self.compute_angles_from_point(x, y, z)

                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
                    self.get_logger().warn(f"TF transform failed: {e}")
                    go_home = True

        # Fallback: raw pixel (if 3D not available)
        elif self.target_pixel is not None:
            x = self.target_pixel.point.x
            y = self.target_pixel.point.y
            z = self.target_pixel.point.z
            if self.is_invalid(x, y, z):
                go_home = True
            else:
                self.desired_pan, self.desired_tilt = self.compute_angles_from_point(x, y, z)

        # Go home if invalid
        if go_home:
            self.desired_pan = self.pan_home
            self.desired_tilt = self.tilt_home

        # Publish joint state
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = ['pan_joint', 'tilt_joint']
        js_msg.position = [self.deg2rad(self.desired_pan), self.deg2rad(self.desired_tilt)]
        self.joint_pub.publish(js_msg)

        # Publish servo angles
        self.pub_servo1.publish(Float32(data=float(self.desired_pan + self.pan_offset)))
        self.pub_servo2.publish(Float32(data=float(self.desired_tilt + self.tilt_offset)))


def main(args=None):
    rclpy.init(args=args)
    node = ServoTargetingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()