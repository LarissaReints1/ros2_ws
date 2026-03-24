import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import yaml
import time
import os
from ament_index_python.packages import get_package_share_directory



class AngleTestPublisher(Node):
	def __init__(self, yaml_file):
		super().__init__('angle_test_publisher')
		self.publisher1_ = self.create_publisher(Float32, 'servo1_angle', 10)
		self.publisher2_ = self.create_publisher(Float32, 'servo2_angle', 10)
		self.yaml_file = yaml_file
		self.angles = self.load_angles()
		self.timer_period = 3.0
		self.index = 0
		self.timer = self.create_timer(self.timer_period, self.publish_next_angle)
		
	def load_angles(self):
		if not os.path.exists(self.yaml_file):
			self.get_logger().error(f"YAML file is not found: {self.yaml_file}")
			return []
		with open(self.yaml_file, 'r') as f:
			data = yaml.safe_load(f)
		return data.get('test_angles',[])
		
	def publish_next_angle(self):
		if not self.angles:
			return
		angle = self.angles[self.index]
		msg = Float32()
		msg = Float32()
		msg.data = angle
		self.publisher1_.publish(msg)
		self.publisher2_.publish(msg)
		self.get_logger().info(f"Published angle: {angle}")
		self.index = (self.index + 1) % len(self.angles)
		
def main(args=None):
	rclpy.init(args=args)

	yaml_file = os.path.join(get_package_share_directory('actuation_pkg'), 'config', 'test_angles.yaml')
	node = AngleTestPublisher(yaml_file)
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()
		
if __name__ == '__main__':
	main()
