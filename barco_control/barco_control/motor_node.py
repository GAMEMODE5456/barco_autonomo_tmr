import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import time

class MotorNode(Node):
    """
    Nodo de control de motor (un solo brushless).

    ARQUITECTURA:
      navigation_node → publica 'motor_speed' (Float32, -1.0..1.0)
      motor_node      → convierte y publica 'motor_speeds' (Float32MultiArray, [velocidad])
      esp_bridge_node → suscribe 'motor_speeds', envía "MOT:0.50" al ESP32
      ESP32           → recibe serial, controla ESC via PCA9685 CH0

    El giro se maneja exclusivamente con el servo del timón (rango ±60°).
    """

    def __init__(self):
        super().__init__('motor_node')

        # Parámetros
        self.declare_parameter('max_speed',     1.0)
        self.declare_parameter('watchdog_secs', 1.0)

        self.max_speed  = self.get_parameter('max_speed').value
        self.watchdog_s = self.get_parameter('watchdog_secs').value

        # Suscripciones
        self.create_subscription(
            Float32, 'motor_speed',
            self.cb_speed, 10)

        # Publisher → esp_bridge_node lo lee
        self.speeds_pub = self.create_publisher(
            Float32MultiArray, 'motor_speeds', 10)

        # Estado interno
        self.speed = 0.0   # velocidad base [-1, 1]
        self.last_cmd_time = time.time()

        # Watchdog: si no hay comandos → stop
        self.create_timer(0.5, self.watchdog_check)
        self.create_timer(0.1, self.publish_speeds)   # 10 Hz

        self.get_logger().info("MotorNode listo (un brushless)")

    def cb_speed(self, msg: Float32):
     self.speed = max(0.0,  # sin reversa
                     min(self.max_speed, msg.data))

    def publish_speeds(self):
        """Publica la velocidad del único motor."""
        msg = Float32MultiArray()
        msg.data = [float(self.speed)]
        self.speeds_pub.publish(msg)

    def watchdog_check(self):
        if time.time() - self.last_cmd_time > self.watchdog_s:
            if abs(self.speed) > 0.0:
                self.speed = 0.0
                self.get_logger().warn("Watchdog: sin comandos → motor stop")


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
