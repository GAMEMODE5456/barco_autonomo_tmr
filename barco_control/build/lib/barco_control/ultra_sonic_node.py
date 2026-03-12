import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

class UltrasonicNode(Node):

    def __init__(self):
        super().__init__('ultra_sonic_node')
        self.publisher_ = self.create_publisher(Float32, 'obstacle_distance', 10)

        # Configuración de pines (ejemplo con un sensor HC-SR04)
        self.TRIG = 23
        self.ECHO = 24
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)

        self.timer = self.create_timer(0.5, self.measure_distance)

    def measure_distance(self):
        GPIO.output(self.TRIG, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, False)

        start = time.time()
        while GPIO.input(self.ECHO) == 0:
            start = time.time()
        while GPIO.input(self.ECHO) == 1:
            end = time.time()

        duration = end - start
        distance = (duration * 34300) / 2  # velocidad del sonido en cm/s

        msg = Float32()
        msg.data = distance
        self.publisher_.publish(msg)
        self.get_logger().info(f"Distancia: {distance:.2f} cm")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    rclpy.spin(node)
    node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()
