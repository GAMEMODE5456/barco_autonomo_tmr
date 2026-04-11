import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import time

class MotorNode(Node):
    """
    Nodo de control de motores.

    ARQUITECTURA CORRECTA:
      navigation_node → publica 'motor_speed' (Float32, -1.0..1.0)
      motor_node      → convierte y publica 'motor_speeds' (Float32MultiArray)
      esp_bridge_node → suscribe 'motor_speeds', envía "MOT:L:X,R:X" al ESP32
      ESP32           → recibe serial, controla ESC via PCA9685 CH0

    Este nodo NO abre el serial — eso es exclusivo de esp_bridge_node.
    Aquí se maneja la lógica de diferencial (girar con motores independientes).
    """

    def __init__(self):
        super().__init__('motor_node')

        # Parámetros
        self.declare_parameter('max_speed',      1.0)
        self.declare_parameter('turn_factor',    0.5)  # cuánto reduce el motor interior al girar
        self.declare_parameter('watchdog_secs',  1.0)  # segundos sin comando → stop

        self.max_speed   = self.get_parameter('max_speed').value
        self.turn_factor = self.get_parameter('turn_factor').value
        self.watchdog_s  = self.get_parameter('watchdog_secs').value

        # Suscripciones
        self.create_subscription(
            Float32, 'motor_speed',
            self.cb_speed, 10)

        self.create_subscription(
            Float32, 'rudder_angle',
            self.cb_rudder, 10)

        # Publisher → esp_bridge_node lo lee
        self.speeds_pub = self.create_publisher(
            Float32MultiArray, 'motor_speeds', 10)

        # Estado interno
        self.speed  = 0.0   # velocidad base [-1, 1]
        self.rudder = 0.0   # ángulo timón [-45, 45]
        self.last_cmd_time = time.time()

        # Watchdog: si no hay comandos → stop
        self.create_timer(0.5, self.watchdog_check)
        self.create_timer(0.1, self.publish_speeds)   # 10 Hz

        self.get_logger().info("MotorNode listo")

    def cb_speed(self, msg: Float32):
        self.speed = max(-self.max_speed,
                         min(self.max_speed, msg.data))
        self.last_cmd_time = time.time()

    def cb_rudder(self, msg: Float32):
        self.rudder = max(-45.0, min(45.0, msg.data))
        self.last_cmd_time = time.time()

    def publish_speeds(self):
        """
        Calcula velocidad diferencial según timón.
        Si el timón va a la derecha, el motor derecho frena un poco
        para ayudar al giro (complementa al servo del timón).
        """
        left  = self.speed
        right = self.speed

        if abs(self.speed) > 0.05:   # solo diferencial si hay movimiento
            # Normalizar rudder a [-1, 1]
            rudder_norm = self.rudder / 45.0
            if rudder_norm > 0:      # girar derecha → frenar motor derecho
                right *= (1.0 - abs(rudder_norm) * self.turn_factor)
            else:                    # girar izquierda → frenar motor izquierdo
                left  *= (1.0 - abs(rudder_norm) * self.turn_factor)

        msg = Float32MultiArray()
        msg.data = [float(left), float(right)]
        self.speeds_pub.publish(msg)

    def watchdog_check(self):
        if time.time() - self.last_cmd_time > self.watchdog_s:
            if abs(self.speed) > 0.0:
                self.speed = 0.0
                self.get_logger().warn("Watchdog: sin comandos → motores stop")


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
