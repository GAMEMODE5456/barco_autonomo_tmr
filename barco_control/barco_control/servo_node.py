import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ServoNode(Node):
    """
    Nodo del timón (servo).

    ARQUITECTURA CORRECTA:
      navigation_node → publica 'rudder_angle' (Float32, grados -45..45)
      servo_node      → suscribe 'rudder_angle', valida y re-publica
      esp_bridge_node → suscribe 'rudder_angle', envía "SRV:XX.X" al ESP32
      ESP32           → recibe serial, mueve servo via PCA9685 CH1

    Este nodo NO accede al PCA9685 directamente porque el PCA9685
    está conectado al ESP32, no a la Raspberry Pi.

    El nodo agrega:
      - Validación de rango
      - Suavizado de movimiento (filtro de paso bajo)
      - Logging del estado actual
    """

    def __init__(self):
        super().__init__('servo_node')

        # Parámetros configurables
        self.declare_parameter('min_angle', -45.0)
        self.declare_parameter('max_angle',  45.0)
        self.declare_parameter('smooth_factor', 0.3)  # 0=sin suavizado, 1=muy suave

        self.min_angle     = self.get_parameter('min_angle').value
        self.max_angle     = self.get_parameter('max_angle').value
        self.smooth_factor = self.get_parameter('smooth_factor').value

        self.current_angle = 0.0   # posición actual (suavizada)

        # Suscripción: recibe ángulo deseado
        self.create_subscription(
            Float32, 'rudder_angle',
            self.cb_rudder, 10)

        # Publisher: ángulo validado y suavizado
        # esp_bridge_node también está suscrito a 'rudder_angle'
        # así que este nodo solo hace validación y logging
        self.angle_pub = self.create_publisher(
            Float32, 'rudder_angle_actual', 10)

        # Timer para publicar estado periódicamente
        self.create_timer(0.5, self.publish_state)

        self.get_logger().info(
            f"ServoNode listo — rango [{self.min_angle}°, {self.max_angle}°]")

    def cb_rudder(self, msg: Float32):
        target = float(msg.data)

        # Validar rango
        target = max(self.min_angle, min(self.max_angle, target))

        # Filtro de paso bajo — suaviza movimientos bruscos
        self.current_angle = (
            self.smooth_factor * self.current_angle +
            (1.0 - self.smooth_factor) * target
        )

        self.get_logger().info(
            f"Timon: objetivo={target:.1f}° actual={self.current_angle:.1f}°")

    def publish_state(self):
        msg = Float32()
        msg.data = self.current_angle
        self.angle_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
