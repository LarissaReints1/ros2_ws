import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import Jetson.GPIO as GPIO
import time

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # PWM pins
        self.servo1_pin = 32  # PWM0
        self.servo2_pin = 33  # PWM1

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servo1_pin, GPIO.OUT)
        GPIO.setup(self.servo2_pin, GPIO.OUT)

        # Set frequency for servos (typical 50Hz)
        self.pwm1 = GPIO.PWM(self.servo1_pin, 50)
        self.pwm2 = GPIO.PWM(self.servo2_pin, 50)

        self.pwm1.start(0)
        time.sleep(0.1)
        self.pwm2.start(0)

        # Subscribers for servo angles
        self.create_subscription(Float32, 'servo1_angle', self.servo1_callback, 10)
        self.create_subscription(Float32, 'servo2_angle', self.servo2_callback, 10)

        self.get_logger().info("Servo Controller Node Started")

    def angle_to_duty_cycle(self, angle):
        # Convert angle (0-180) to duty cycle (2.5-12.5)
        duty = 2.5 + (angle / 180.0) * 10
        return duty

    def servo1_callback(self, msg):
        duty = self.angle_to_duty_cycle(msg.data)
        self.pwm1.ChangeDutyCycle(duty)
        self.get_logger().info(f"Servo1 set to {msg.data}°")

    def servo2_callback(self, msg):
        duty = self.angle_to_duty_cycle(msg.data)
        self.pwm2.ChangeDutyCycle(duty)
        self.get_logger().info(f"Servo2 set to {msg.data}°")

    def destroy_node(self):
        self.pwm1.stop()
        self.pwm2.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
