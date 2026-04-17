import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool, String
from sensor_msgs.msg import NavSatFix, Imu
import time

# ─────────────────────────────────────────────────────────
# ESTADOS
# ─────────────────────────────────────────────────────────
class Estado:
    BUSCAR    = 'BUSCAR'
    APUNTAR   = 'APUNTAR'
    ACERCARSE = 'ACERCARSE'
    RECOGER   = 'RECOGER'
    GIRAR     = 'GIRAR'     # antes EVADIR — ahora solo corrige rumbo ante boyas


class NavigationNode(Node):
    """
    Máquina de estados para navegación autónoma.
    Un solo motor brushless unidireccional + servo timón ±60°.

    Lógica de boyas:
      Las boyas delimitan la zona — el barco NO debe salir.
      Cuando detecta boya → GIRAR hacia el interior (opuesto a la boya).
      No retrocede, solo corrige rumbo y sigue avanzando.

    Topics suscritos:
      target_detected  (Bool)              — sargaso verde o café
      target_position  (Float32MultiArray) — [x, y, area]
      boya_detected    (Bool)              — boya roja/naranja
      boya_position    (Float32MultiArray) — [x, y, area]
      obstacle_distance(Float32)
      imu_data         (Imu)
      gps_data         (NavSatFix)

    Topics publicados:
      motor_speed    (Float32)
      rudder_angle   (Float32)
      conveyor_power (Bool)
      nav_state      (String)
    """

    def __init__(self):
        super().__init__('navigation_node')

        # ── Parámetros ────────────────────────────────────
        self.declare_parameter('search_turn_speed',  0.25)
        self.declare_parameter('approach_speed',     0.40)
        self.declare_parameter('collect_speed',      0.20)
        self.declare_parameter('max_rudder',         55.0)
        self.declare_parameter('rudder_kp',          55.0)
        self.declare_parameter('center_threshold',   0.15)
        self.declare_parameter('collect_area',       0.15)
        self.declare_parameter('obstacle_stop_cm',   30.0)
        self.declare_parameter('collect_time',        3.0)
        self.declare_parameter('search_timeout',      8.0)
        self.declare_parameter('boya_threshold',      0.6)  # x_norm para considerar boya "muy cerca"
        self.declare_parameter('girar_time',          1.5)  # segundos girando para esquivar boya

        self.search_turn    = self.get_parameter('search_turn_speed').value
        self.approach_spd   = self.get_parameter('approach_speed').value
        self.collect_spd    = self.get_parameter('collect_speed').value
        self.max_rudder     = self.get_parameter('max_rudder').value
        self.rudder_kp      = self.get_parameter('rudder_kp').value
        self.center_thr     = self.get_parameter('center_threshold').value
        self.collect_area   = self.get_parameter('collect_area').value
        self.obs_stop       = self.get_parameter('obstacle_stop_cm').value
        self.collect_time   = self.get_parameter('collect_time').value
        self.search_timeout = self.get_parameter('search_timeout').value
        self.boya_thr       = self.get_parameter('boya_threshold').value
        self.girar_time     = self.get_parameter('girar_time').value

        # ── Estado ────────────────────────────────────────
        self.estado        = Estado.BUSCAR
        self.estado_previo = None
        self.estado_inicio = time.time()

        # Sensores
        self.target_detected = False
        self.target_x        = 0.0
        self.target_y        = 0.0
        self.target_area     = 0.0
        self.boya_detected   = False
        self.boya_x          = 0.0
        self.boya_area       = 0.0
        self.obstacle_dist   = 9999.0
        self.imu_yaw         = 0.0
        self.gps_lat         = None
        self.gps_lon         = None
        self.conveyor_activa = False

        # Timeout danger para evitar falsos positivos sin cámara
        self.last_boya_time   = time.time() - 10.0
        self.last_target_time = time.time() - 10.0
        self.SENSOR_TIMEOUT   = 1.0  # segundos sin mensaje → resetear

        # Dirección de búsqueda
        self._search_dir = 1.0

        # ── Suscripciones ─────────────────────────────────
        self.create_subscription(
            Bool, 'target_detected', self.cb_target_detected, 10)
        self.create_subscription(
            Float32MultiArray, 'target_position', self.cb_target_position, 10)
        self.create_subscription(
            Bool, 'boya_detected', self.cb_boya_detected, 10)
        self.create_subscription(
            Float32MultiArray, 'boya_position', self.cb_boya_position, 10)
        self.create_subscription(
            Float32, 'obstacle_distance', self.cb_obstacle, 10)
        self.create_subscription(
            Imu, 'imu_data', self.cb_imu, 10)
        self.create_subscription(
            NavSatFix, 'gps_data', self.cb_gps, 10)

        # ── Publishers ────────────────────────────────────
        self.motor_pub    = self.create_publisher(Float32, 'motor_speed',    10)
        self.rudder_pub   = self.create_publisher(Float32, 'rudder_angle',   10)
        self.conveyor_pub = self.create_publisher(Bool,    'conveyor_power', 10)
        self.estado_pub   = self.create_publisher(String,  'nav_state',      10)

        self.create_timer(0.1, self.control_loop)

        self._set_conveyor(True)
        self.get_logger().info("NavigationNode listo — estado: BUSCAR")

    # ─────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────

    def cb_target_detected(self, msg: Bool):
        self.target_detected = msg.data
        if msg.data:
            self.last_target_time = time.time()

    def cb_target_position(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.target_x    = msg.data[0]
            self.target_y    = msg.data[1]
            self.target_area = msg.data[2]

    def cb_boya_detected(self, msg: Bool):
        self.boya_detected = msg.data
        if msg.data:
            self.last_boya_time = time.time()

    def cb_boya_position(self, msg: Float32MultiArray):
        if len(msg.data) >= 1:
            self.boya_x = msg.data[0]
        if len(msg.data) >= 3:
            self.boya_area = msg.data[2]

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
        vel = max(0.0, min(1.0, vel))  # unidireccional
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

    def _boya_activa(self) -> bool:
        """True si hay boya detectada recientemente (con timeout)."""
        if time.time() - self.last_boya_time > self.SENSOR_TIMEOUT:
            self.boya_detected = False
        return self.boya_detected

    def _target_activo(self) -> bool:
        """True si hay sargaso detectado recientemente (con timeout)."""
        if time.time() - self.last_target_time > self.SENSOR_TIMEOUT:
            self.target_detected = False
        return self.target_detected

    # ─────────────────────────────────────────────────────
    # Loop principal
    # ─────────────────────────────────────────────────────

    def control_loop(self):

        # Actualizar timeouts
        boya_cerca   = self._boya_activa()
        target_cerca = self._target_activo()

        # Prioridad 1: boya detectada → GIRAR (no salir de zona)
        if boya_cerca and self.estado != Estado.GIRAR:
            self.get_logger().warn(
                f"BOYA detectada x={self.boya_x:+.2f} → GIRAR")
            self._cambiar_estado(Estado.GIRAR)

        # Prioridad 2: obstáculo físico muy cerca → GIRAR
        if (self.obstacle_dist < self.obs_stop
                and self.estado != Estado.GIRAR):
            self.get_logger().warn(
                f"Obstáculo a {self.obstacle_dist:.1f}cm → GIRAR")
            self._cambiar_estado(Estado.GIRAR)

        if self.estado == Estado.BUSCAR:
            self._estado_buscar(target_cerca)
        elif self.estado == Estado.APUNTAR:
            self._estado_apuntar(target_cerca)
        elif self.estado == Estado.ACERCARSE:
            self._estado_acercarse(target_cerca)
        elif self.estado == Estado.RECOGER:
            self._estado_recoger()
        elif self.estado == Estado.GIRAR:
            self._estado_girar(boya_cerca)

    # ─────────────────────────────────────────────────────
    # ESTADOS
    # ─────────────────────────────────────────────────────

    def _estado_buscar(self, target_cerca: bool):
        """
        Avanza girando con timón al máximo para barrer la zona.
        Alterna dirección cada search_timeout segundos.
        """
        if target_cerca:
            self._cambiar_estado(Estado.APUNTAR)
            return

        self._set_motor(self.search_turn)
        self._set_rudder(self.max_rudder * self._search_dir)

        if self._tiempo_en_estado() > self.search_timeout:
            self._search_dir *= -1
            self.estado_inicio = time.time()
            self.get_logger().warn("Sin sargaso — cambiando dirección de búsqueda")

    def _estado_apuntar(self, target_cerca: bool):
        """Corrige el timón para centrar el sargaso en el frame."""
        if not target_cerca:
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

    def _estado_acercarse(self, target_cerca: bool):
        """Avanza hacia el sargaso corrigiendo el timón."""
        if not target_cerca:
            self._cambiar_estado(Estado.APUNTAR)
            return

        if self.target_area >= self.collect_area:
            self._cambiar_estado(Estado.RECOGER)
            return

        rudder = self._rudder_desde_camara()
        self._set_rudder(rudder)

        factor = 1.0 - abs(self.target_x) * 0.4
        spd    = self.approach_spd * factor
        self._set_motor(spd)

        self.get_logger().debug(
            f"ACERCARSE area={self.target_area:.3f} "
            f"x={self.target_x:.2f} spd={spd:.2f}")

    def _estado_recoger(self):
        """Avanza despacio para recoger el sargaso con la banda."""
        rudder = self._rudder_desde_camara()
        self._set_rudder(rudder)
        self._set_motor(self.collect_spd)

        if self._tiempo_en_estado() >= self.collect_time:
            self.get_logger().info(
                f"Recolección completada ({self.collect_time}s) → BUSCAR")
            self._stop()
            self._cambiar_estado(Estado.BUSCAR)

    def _estado_girar(self, boya_cerca: bool):
        """
        Gira hacia el interior de la zona cuando detecta una boya.
        NO retrocede — solo corrige el rumbo girando en sentido opuesto a la boya.

        Lógica:
          boya a la izquierda  (boya_x < 0) → girar a la derecha (rudder +)
          boya a la derecha    (boya_x > 0) → girar a la izquierda (rudder -)
          boya al centro       (boya_x ≈ 0) → girar a la derecha por defecto
        """
        t = self._tiempo_en_estado()

        # Determinar dirección de giro (opuesta a la boya)
        if self.boya_x <= 0:
            rudder_giro = self.max_rudder    # girar derecha
        else:
            rudder_giro = -self.max_rudder   # girar izquierda

        if t < self.girar_time:
            # Girar avanzando despacio hacia el interior
            self._set_motor(self.search_turn * 0.8)
            self._set_rudder(rudder_giro)
            self.get_logger().debug(
                f"GIRAR t={t:.1f}s boya_x={self.boya_x:.2f} "
                f"rudder={rudder_giro:.0f}°")
        else:
            # Terminó el giro — volver al estado anterior
            self._stop()
            estado_retorno = self.estado_previo if self.estado_previo else Estado.BUSCAR

            # Si la boya sigue ahí, seguir girando
            if boya_cerca:
                self.estado_inicio = time.time()  # reiniciar timer
                self.get_logger().warn("Boya aún visible — continuando giro")
            else:
                self._cambiar_estado(estado_retorno)
                self.get_logger().info(
                    f"Boya esquivada → {estado_retorno}")


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
