import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ServoNode(Node):
    """
    Nodo del timón (servo) — rango ampliado a ±60°.

    ARQUITECTURA:
      navigation_node → publica 'rudder_angle' (Float32, grados -60..60)
      servo_node      → suscribe 'rudder_angle', valida y re-publica
      esp_bridge_node → suscribe 'rudder_angle', envía "SRV:XX.X" al ESP32
      ESP32           → recibe serial, mueve servo via PCA9685 CH1

    El rango se amplió de ±45° a ±60° para mayor maniobrabilidad.
    """

    def __init__(self):
        super().__init__('servo_node')

        # Parámetros configurables — rango ampliado a ±60°
        self.declare_parameter('min_angle', -60.0)
        self.declare_parameter('max_angle',  60.0)
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
        self.angle_pub = self.create_publisher(
            Float32, 'rudder_angle_actual', 10)

        # Timer para publicar estado periódicamente
        self.create_timer(0.5, self.publish_state)

        self.get_logger().info(
            f"ServoNode listo — rango [{self.min_angle}°, {self.max_angle}°]")

    def cb_rudder(self, msg: Float32):
        target = float(msg.data)

        # Validar rango ±60°
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
