import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class ArduinoServoNode(Node):
    def __init__(self):
        super().__init__('arduino_servo_node')

        # Serial setup
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)  # allow Arduino to reset

        # Current servo values in microseconds
        self.servo1_us = 1500
        self.servo2_us = 1500

        # Subscribers
        self.create_subscription(Float32, 'servo1_angle', self.servo1_callback, 10)
        self.create_subscription(Float32, 'servo2_angle', self.servo2_callback, 10)
        self.create_subscription(Float32, 'servo_both_angle', self.servo_both_callback, 10)

        self.get_logger().info("Arduino Servo Node started")

    def angle_to_us(self, angle: float) -> int:
        """Convert 0–180° angle to 500–2500 µs pulse width"""
        return int(500 + (angle / 180.0) * 2000)

    def send_servo_values(self):
        """Send the current servo microsecond values to Arduino"""
        command = f"{self.servo1_us},{self.servo2_us}\n"
        self.ser.write(command.encode())
        self.get_logger().debug(f"Sent to Arduino: {command.strip()}")

    def servo1_callback(self, msg: Float32):
        self.servo1_us = self.angle_to_us(msg.data)
        self.send_servo_values()

    def servo2_callback(self, msg: Float32):
        self.servo2_us = self.angle_to_us(msg.data)
        self.send_servo_values()

    def servo_both_callback(self, msg: Float32):
        value_us = self.angle_to_us(msg.data)
        self.servo1_us = value_us
        self.servo2_us = value_us
        self.send_servo_values()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()