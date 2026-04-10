import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import time

class ConveyorNode(Node):
    """
    Nodo de la banda transportadora.

    ARQUITECTURA:
      navigation_node  → publica 'conveyor_power' (Bool)
      conveyor_node    → valida, agrega lógica de tiempo, re-publica
      esp_bridge_node  → suscribe 'conveyor_power' → envía "CONV:ON/OFF" al ESP32
      ESP32            → PCA9685 CH2 → L298N → motorreductor amarillo

    Lógica:
      - Al activarse corre la banda por 'run_duration' segundos
      - Luego se detiene automáticamente (evita que quede encendida)
      - Se puede activar manualmente con duración indefinida poniendo run_duration=0
      - Publica 'conveyor_status' con el estado actual
    """

    def __init__(self):
        super().__init__('conveyor_node')

        # Parámetros
        self.declare_parameter('run_duration', 5.0)   # segundos que corre al activarse
                                                       # 0.0 = indefinido hasta OFF
        self.declare_parameter('cooldown',     2.0)   # segundos mínimos entre activaciones

        self.run_duration = self.get_parameter('run_duration').value
        self.cooldown     = self.get_parameter('cooldown').value

        # Estado
        self.is_running       = False
        self.start_time       = 0.0
        self.last_stop_time   = 0.0

        # Suscripción: recibe orden de encender/apagar
        self.create_subscription(
            Bool, 'conveyor_power',
            self.cb_power, 10)

        # Publishers
        self.power_pub  = self.create_publisher(Bool,    'conveyor_power',  10)
        self.status_pub = self.create_publisher(Bool,    'conveyor_status', 10)
        self.speed_pub  = self.create_publisher(Float32, 'conveyor_speed',  10)

        # Timer de control (verifica timeout de duración)
        self.create_timer(0.2, self.control_loop)

        self.get_logger().info(
            f"ConveyorNode listo — duracion={self.run_duration}s "
            f"cooldown={self.cooldown}s")

    # ────────────────────────────────────────────
    # Callback de activación
    # ────────────────────────────────────────────

    def cb_power(self, msg: Bool):
        if msg.data:
            self.activate()
        else:
            self.deactivate()

    def activate(self):
        now = time.time()

        # Verificar cooldown
        if now - self.last_stop_time < self.cooldown:
            remaining = self.cooldown - (now - self.last_stop_time)
            self.get_logger().warn(
                f"Banda en cooldown — espera {remaining:.1f}s")
            return

        if self.is_running:
            self.get_logger().debug("Banda ya está corriendo")
            return

        self.is_running = True
        self.start_time = now

        # Publicar ON para que esp_bridge lo envíe al ESP32
        out = Bool()
        out.data = True
        self.power_pub.publish(out)

        self.get_logger().info(
            f"Banda ACTIVADA"
            + (f" — correrá {self.run_duration}s"
               if self.run_duration > 0 else " — modo indefinido"))

    def deactivate(self):
        if not self.is_running:
            return

        self.is_running     = False
        self.last_stop_time = time.time()

        # Publicar OFF para que esp_bridge lo envíe al ESP32
        out = Bool()
        out.data = False
        self.power_pub.publish(out)

        self.get_logger().info("Banda DESACTIVADA")

    # ────────────────────────────────────────────
    # Loop de control — verifica timeout
    # ────────────────────────────────────────────

    def control_loop(self):
        now = time.time()

        # Auto-apagar si se cumplió la duración
        if (self.is_running
                and self.run_duration > 0
                and (now - self.start_time) >= self.run_duration):
            self.get_logger().info(
                f"Banda cumplió {self.run_duration}s → apagando")
            self.deactivate()

        # Publicar estado actual
        status = Bool()
        status.data = self.is_running
        self.status_pub.publish(status)

        # Publicar velocidad (1.0 = corriendo, 0.0 = detenida)
        speed = Float32()
        speed.data = 1.0 if self.is_running else 0.0
        self.speed_pub.publish(speed)


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
