import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool, String
from sensor_msgs.msg import NavSatFix, Imu
import time

# ─────────────────────────────────────────────────────────
# ESTADOS DE LA MÁQUINA
# ─────────────────────────────────────────────────────────
class Estado:
    BUSCAR    = 'BUSCAR'
    APUNTAR   = 'APUNTAR'
    ACERCARSE = 'ACERCARSE'
    RECOGER   = 'RECOGER'
    EVADIR    = 'EVADIR'


class NavigationNode(Node):
    """
    Máquina de estados para navegación autónoma del barco.
    Un solo motor brushless + servo timón ±60°.

    Publica:
      'motor_speed'   (Float32)  → motor_node
      'rudder_angle'  (Float32)  → servo_node + esp_bridge_node
      'conveyor_power'(Bool)
      'nav_state'     (String)
    """

    def __init__(self):
        super().__init__('navigation_node')

        # ── Parámetros ────────────────────────────────────
        self.declare_parameter('search_turn_speed',  0.25)
        self.declare_parameter('approach_speed',     0.45)
        self.declare_parameter('collect_speed',      0.20)
        self.declare_parameter('max_rudder',         55.0)
        self.declare_parameter('rudder_kp',          55.0)
        self.declare_parameter('center_threshold',   0.12)
        self.declare_parameter('collect_area',       0.15)
        self.declare_parameter('obstacle_stop_cm',   40.0)
        self.declare_parameter('obstacle_evade_cm',  80.0)
        self.declare_parameter('collect_time',        3.0)
        self.declare_parameter('search_timeout',      8.0)

        self.search_turn    = self.get_parameter('search_turn_speed').value
        self.approach_spd   = self.get_parameter('approach_speed').value
        self.collect_spd    = self.get_parameter('collect_speed').value
        self.max_rudder     = self.get_parameter('max_rudder').value
        self.rudder_kp      = self.get_parameter('rudder_kp').value
        self.center_thr     = self.get_parameter('center_threshold').value
        self.collect_area   = self.get_parameter('collect_area').value
        self.obs_stop       = self.get_parameter('obstacle_stop_cm').value
        self.obs_evade      = self.get_parameter('obstacle_evade_cm').value
        self.collect_time   = self.get_parameter('collect_time').value
        self.search_timeout = self.get_parameter('search_timeout').value

        # ── Estado ────────────────────────────────────────
        self.estado        = Estado.BUSCAR
        self.estado_previo = None
        self.estado_inicio = time.time()

        # Sensores
        self.target_detected = False
        self.target_x        = 0.0
        self.target_y        = 0.0
        self.target_area     = 0.0
        self.danger_detected = False
        self.danger_x        = 0.0
        self.obstacle_dist   = 9999.0
        self.imu_yaw         = 0.0
        self.gps_lat         = None
        self.gps_lon         = None
        self.conveyor_activa = False

        # Dirección de búsqueda (alterna cada timeout)
        self._search_dir = 1.0   # +1 derecha, -1 izquierda

        # ── Suscripciones ─────────────────────────────────
        self.create_subscription(
            Bool, 'target_detected', self.cb_detected, 10)
        self.create_subscription(
            Float32MultiArray, 'target_position', self.cb_position, 10)
        self.create_subscription(
            Bool, 'danger_detected', self.cb_danger_detected, 10)
        self.create_subscription(
            Float32MultiArray, 'danger_position', self.cb_danger_position, 10)
        self.create_subscription(
            Float32, 'obstacle_distance', self.cb_obstacle, 10)
        self.create_subscription(
            Imu, 'imu_data', self.cb_imu, 10)
        self.create_subscription(
            NavSatFix, 'gps_data', self.cb_gps, 10)

        # ── Publishers ────────────────────────────────────
        self.motor_pub    = self.create_publisher(
            Float32, 'motor_speed',    10)
        self.rudder_pub   = self.create_publisher(
            Float32, 'rudder_angle',   10)
        self.conveyor_pub = self.create_publisher(
            Bool,    'conveyor_power', 10)
        self.estado_pub   = self.create_publisher(
            String,  'nav_state',      10)

        self.create_timer(0.1, self.control_loop)

        self._set_conveyor(True)
        self.get_logger().info("NavigationNode listo — estado: BUSCAR")

    # ─────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────

    def cb_detected(self, msg: Bool):
        self.target_detected = msg.data

    def cb_position(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.target_x    = msg.data[0]
            self.target_y    = msg.data[1]
            self.target_area = msg.data[2]

    def cb_danger_detected(self, msg: Bool):
        self.danger_detected = msg.data

    def cb_danger_position(self, msg: Float32MultiArray):
        if len(msg.data) >= 1:
            self.danger_x = msg.data[0]

    def cb_obstacle(self, msg: Float32):
        self.obstacle_dist = msg.data

    def cb_imu(self, msg: Imu):
        self.imu_yaw = msg.orientation.z

    def cb_gps(self, msg: NavSatFix):
        self.gps_lat = msg.latitude
        self.gps_lon = msg.longitude

    # ─────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────

    def _set_motor(self, vel: float):
        vel = max(-1.0, min(1.0, vel))
        msg = Float32()
        msg.data = vel
        self.motor_pub.publish(msg)

    def _set_rudder(self, angle: float):
        angle = max(-self.max_rudder, min(self.max_rudder, angle))
        msg = Float32()
        msg.data = angle
        self.rudder_pub.publish(msg)

    def _set_conveyor(self, on: bool):
        if self.conveyor_activa != on:
            self.conveyor_activa = on
            msg = Bool()
            msg.data = on
            self.conveyor_pub.publish(msg)

    def _stop(self):
        self._set_motor(0.0)
        self._set_rudder(0.0)

    def _cambiar_estado(self, nuevo: str):
        if nuevo != self.estado:
            self.get_logger().info(f"Estado: {self.estado} → {nuevo}")
            self.estado_previo = self.estado
            self.estado        = nuevo
            self.estado_inicio = time.time()
            msg = String()
            msg.data = nuevo
            self.estado_pub.publish(msg)

    def _tiempo_en_estado(self) -> float:
        return time.time() - self.estado_inicio

    def _rudder_desde_camara(self) -> float:
        rudder = self.target_x * self.rudder_kp
        return max(-self.max_rudder, min(self.max_rudder, rudder))

    # ─────────────────────────────────────────────────────
    # Loop principal
    # ─────────────────────────────────────────────────────

    def control_loop(self):

        # Prioridad 1: sargaso rojo → EVADIR
        if self.danger_detected and self.estado != Estado.EVADIR:
            self.get_logger().warn("Sargaso ROJO detectado → EVADIR")
            self._cambiar_estado(Estado.EVADIR)

        # Prioridad 2: obstáculo físico → EVADIR
        if self.obstacle_dist < self.obs_stop and self.estado != Estado.EVADIR:
            self._cambiar_estado(Estado.EVADIR)

        if self.estado == Estado.BUSCAR:
            self._estado_buscar()
        elif self.estado == Estado.APUNTAR:
            self._estado_apuntar()
        elif self.estado == Estado.ACERCARSE:
            self._estado_acercarse()
        elif self.estado == Estado.RECOGER:
            self._estado_recoger()
        elif self.estado == Estado.EVADIR:
            self._estado_evadir()

    # ─────────────────────────────────────────────────────
    # ESTADOS
    # ─────────────────────────────────────────────────────

    def _estado_buscar(self):
        """
        Avanza girando con timón al máximo para barrer el área.
        Con un solo motor no puede girar en el lugar.
        Alterna dirección cada search_timeout segundos.
        """
        if self.target_detected:
            self._cambiar_estado(Estado.APUNTAR)
            return

        self._set_motor(self.search_turn)
        self._set_rudder(self.max_rudder * self._search_dir)

        if self._tiempo_en_estado() > self.search_timeout:
            self._search_dir *= -1   # alternar dirección
            self.estado_inicio = time.time()
            self.get_logger().warn(
                f"Sin objeto — cambiando dirección de búsqueda")

    def _estado_apuntar(self):
        """Corrige el timón para centrar el objeto en el frame."""
        if not self.target_detected:
            self._cambiar_estado(Estado.BUSCAR)
            return

        error_x  = self.target_x
        centrado = abs(error_x) < self.center_thr
        rudder   = self._rudder_desde_camara()

        self._set_rudder(rudder)
        self._set_motor(self.approach_spd * 0.4)

        if centrado:
            self._cambiar_estado(Estado.ACERCARSE)

        self.get_logger().debug(
            f"APUNTAR error_x={error_x:.2f} rudder={rudder:.1f}°")

    def _estado_acercarse(self):
        """Avanza hacia el objeto corrigiendo el timón continuamente."""
        if not self.target_detected:
            self._cambiar_estado(Estado.APUNTAR)
            return

        if self.target_area >= self.collect_area:
            self._cambiar_estado(Estado.RECOGER)
            return

        rudder = self._rudder_desde_camara()
        self._set_rudder(rudder)

        factor = 1.0 - abs(self.target_x) * 0.4
        spd    = self.approach_spd * factor

        if self.obstacle_dist < self.obs_evade:
            reduccion = 1.0 - (
                (self.obs_evade - self.obstacle_dist) / self.obs_evade)
            spd *= max(0.1, reduccion)

        self._set_motor(spd)

        self.get_logger().debug(
            f"ACERCARSE area={self.target_area:.3f} "
            f"x={self.target_x:.2f} spd={spd:.2f}")

    def _estado_recoger(self):
        """Avanza despacio para recoger el objeto con la banda."""
        rudder = self._rudder_desde_camara()
        self._set_rudder(rudder)
        self._set_motor(self.collect_spd)

        if self._tiempo_en_estado() >= self.collect_time:
            self.get_logger().info(
                f"Recolección completada ({self.collect_time}s) → BUSCAR")
            self._stop()
            self._cambiar_estado(Estado.BUSCAR)

    def _estado_evadir(self):
        """Retrocede y gira para esquivar obstáculo o sargaso rojo."""
        t = self._tiempo_en_estado()

        if t < 0.8:
            self._set_motor(-0.3)
            self._set_rudder(0.0)
        elif t < 1.6:
            self._set_motor(0.3)
            self._set_rudder(self.max_rudder)
        else:
            self._stop()
            estado_retorno = self.estado_previo if self.estado_previo else Estado.BUSCAR
            self._cambiar_estado(estado_retorno)

        self.get_logger().debug(
            f"EVADIR t={t:.1f}s dist={self.obstacle_dist:.1f}cm")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._stop()
    node._set_conveyor(False)
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()
