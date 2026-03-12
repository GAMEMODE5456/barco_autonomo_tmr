import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from adafruit_pca9685 import PCA9685
import busio
from board import SCL, SDA

class ServoNode(Node):

    def __init__(self):
        super().__init__('servo_node')

        # Inicializar I2C y PCA9685
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50  # Frecuencia típica para servos

        # Suscripción al tópico "rudder_angle"
        self.subscription = self.create_subscription(
            Float32,
            'rudder_angle',
            self.listener_callback,
            10
        )

        self.get_logger().info("Nodo del servomotor inicializado")

    def set_servo_angle(self, angle):
        # Convertir ángulo (ej. -45 a +45 grados) a pulso PWM
        # Estos valores dependen del servo y calibración
        min_pulse = 1000  # microsegundos (~1 ms)
        max_pulse = 2000  # microsegundos (~2 ms)
        pulse = int(min_pulse + (angle + 45) * (max_pulse - min_pulse) / 90)

        # Convertir a duty cycle para PCA9685 (12 bits: 0-4095)
        duty_cycle = int(pulse * self.pca.frequency * 4096 / 1_000_000)
        self.pca.channels[0].duty_cycle = duty_cycle

        self.get_logger().info(f"Servo movido a {angle} grados")

    def listener_callback(self, msg):
        angle = msg.data
        self.set_servo_angle(angle)

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
