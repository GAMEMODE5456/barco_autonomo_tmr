import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool, String
from sensor_msgs.msg import NavSatFix, Imu
import math
import time

# ─────────────────────────────────────────────────────────
# ESTADOS DE LA MÁQUINA
# ─────────────────────────────────────────────────────────
class Estado:
    BUSCAR    = 'BUSCAR'     # girando lentamente buscando objeto
    APUNTAR   = 'APUNTAR'   # objeto detectado, centrando en frame
    ACERCARSE = 'ACERCARSE' # objeto centrado, avanzando hacia él
    RECOGER   = 'RECOGER'   # objeto muy cerca, velocidad baja
    EVADIR    = 'EVADIR'    # obstáculo detectado, maniobrando


class NavigationNode(Node):
    """
    Máquina de estados para navegación autónoma del barco.

    Tópicos suscritos:
      'target_detected'   (Bool)              — cámara detectó objeto
      'target_position'   (Float32MultiArray) — [x_norm, y_norm, area_norm]
      'obstacle_distance' (Float32)           — distancia ultrasonido cm
      'imu_data'          (Imu)               — orientación
      'gps_data'          (NavSatFix)         — posición GPS

    Tópicos publicados:
      'motor_speeds'      (Float32MultiArray) — [izq, der] -1..1
      'rudder_angle'      (Float32)           — grados -45..45
      'conveyor_power'    (Bool)              — banda ON/OFF
    """

    def __init__(self):
        super().__init__('navigation_node')

        # ── Parámetros ajustables ──────────────────────────
        self.declare_parameter('search_turn_speed',  0.25)  # velocidad giro búsqueda
        self.declare_parameter('approach_speed',     0.45)  # velocidad de acercamiento
        self.declare_parameter('collect_speed',      0.20)  # velocidad al recoger
        self.declare_parameter('max_rudder',         35.0)  # ángulo máximo timón
        self.declare_parameter('rudder_kp',          40.0)  # ganancia proporcional timón
        self.declare_parameter('center_threshold',   0.12)  # error x para considerar centrado
        self.declare_parameter('collect_area',       0.15)  # área mínima para recoger
        self.declare_parameter('obstacle_stop_cm',   40.0)  # cm para parar por obstáculo
        self.declare_parameter('obstacle_evade_cm',  80.0)  # cm para iniciar evasión
        self.declare_parameter('collect_time',        3.0)  # segundos recogiendo antes de buscar
        self.declare_parameter('search_timeout',     8.0)   # segundos girando sin ver nada

        self.search_turn   = self.get_parameter('search_turn_speed').value
        self.approach_spd  = self.get_parameter('approach_speed').value
        self.collect_spd   = self.get_parameter('collect_speed').value
        self.max_rudder    = self.get_parameter('max_rudder').value
        self.rudder_kp     = self.get_parameter('rudder_kp').value
        self.center_thr    = self.get_parameter('center_threshold').value
        self.collect_area  = self.get_parameter('collect_area').value
        self.obs_stop      = self.get_parameter('obstacle_stop_cm').value
        self.obs_evade     = self.get_parameter('obstacle_evade_cm').value
        self.collect_time  = self.get_parameter('collect_time').value
        self.search_timeout= self.get_parameter('search_timeout').value

        # ── Estado ────────────────────────────────────────
        self.estado          = Estado.BUSCAR
        self.estado_previo   = None
        self.estado_inicio   = time.time()

        # Datos de sensores
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

        # Control
        self.conveyor_activa = False

        # ── Suscripciones ────────────────────────────────
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

        # ── Publishers ───────────────────────────────────
        self.motor_pub    = self.create_publisher(
            Float32MultiArray, 'motor_speeds',   10)
        self.rudder_pub   = self.create_publisher(
            Float32,           'rudder_angle',   10)
        self.conveyor_pub = self.create_publisher(
            Bool,              'conveyor_power', 10)
        self.estado_pub   = self.create_publisher(
            String,            'nav_state',      10)

        # ── Loop de control a 10 Hz ──────────────────────
        self.create_timer(0.1, self.control_loop)

        # Activar banda desde el inicio (siempre activa)
        self._set_conveyor(True)

        self.get_logger().info("NavigationNode listo — estado: BUSCAR")

    # ─────────────────────────────────────────────────────
    # Callbacks de sensores
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
    # Helpers de actuación
    # ─────────────────────────────────────────────────────

    def _set_motors(self, izq: float, der: float):
        izq = max(-1.0, min(1.0, izq))
        der = max(-1.0, min(1.0, der))
        msg = Float32MultiArray()
        msg.data = [izq, der]
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
        self._set_motors(0.0, 0.0)
        self._set_rudder(0.0)

    def _cambiar_estado(self, nuevo: str):
        if nuevo != self.estado:
            self.get_logger().info(
                f"Estado: {self.estado} → {nuevo}")
            self.estado_previo = self.estado
            self.estado        = nuevo
            self.estado_inicio = time.time()
            msg = String()
            msg.data = nuevo
            self.estado_pub.publish(msg)

    def _tiempo_en_estado(self) -> float:
        return time.time() - self.estado_inicio

    # ─────────────────────────────────────────────────────
    # Calcular ángulo de timón por visual servoing
    # Proporcional al error horizontal de la cámara
    # ─────────────────────────────────────────────────────

    def _rudder_desde_camara(self) -> float:
        """
        target_x: -1=izquierda, 0=centro, +1=derecha
        Si el objeto está a la derecha del frame → girar derecha (ángulo +)
        """
        rudder = self.target_x * self.rudder_kp
        return max(-self.max_rudder, min(self.max_rudder, rudder))

    # ─────────────────────────────────────────────────────
    # LOOP DE CONTROL PRINCIPAL
    # ─────────────────────────────────────────────────────

    def control_loop(self):

        # ── Prioridad 1: sargaso rojo cerca → EVADIR ──
        if (self.danger_detected
                and self.estado != Estado.EVADIR):
            self.get_logger().warn("Sargaso ROJO detectado → EVADIR")
            self._cambiar_estado(Estado.EVADIR)

        # ── Prioridad 2: obstáculo físico muy cerca → EVADIR ──
        if (self.obstacle_dist < self.obs_stop
                and self.estado != Estado.EVADIR):
            self._cambiar_estado(Estado.EVADIR)

        # ── Máquina de estados ───────────────────────────
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
        Gira lentamente en el lugar buscando un objeto.
        Motor izq adelante, motor der atrás = giro sobre el eje.
        Alterna dirección cada search_timeout segundos si no ve nada.
        """
        if self.target_detected:
            self._cambiar_estado(Estado.APUNTAR)
            return

        # Girar en el lugar: motor izq adelante, der atrás
        self._set_motors(self.search_turn, -self.search_turn)
        self._set_rudder(0.0)

        # Si lleva mucho tiempo girando sin ver nada,
        # registrar en log (podría alternarse la dirección aquí)
        if self._tiempo_en_estado() > self.search_timeout:
            self.get_logger().warn(
                f"Buscando hace {self.search_timeout}s sin detectar objeto")
            self.estado_inicio = time.time()  # reiniciar timer

    def _estado_apuntar(self):
        """
        Objeto detectado. Corrige el timón para centrarlo en el frame.
        Avanza muy despacio mientras apunta.
        Cuando el error es pequeño → ACERCARSE.
        """
        if not self.target_detected:
            self._cambiar_estado(Estado.BUSCAR)
            return

        error_x = self.target_x   # -1..1
        centrado = abs(error_x) < self.center_thr

        rudder = self._rudder_desde_camara()
        self._set_rudder(rudder)

        if centrado:
            # Avanzar cuando está centrado
            self._set_motors(self.approach_spd * 0.5,
                             self.approach_spd * 0.5)
            self._cambiar_estado(Estado.ACERCARSE)
        else:
            # Girar en el lugar para apuntar
            if error_x > 0:
                self._set_motors(self.search_turn, -self.search_turn * 0.5)
            else:
                self._set_motors(-self.search_turn * 0.5, self.search_turn)

        self.get_logger().debug(
            f"APUNTAR error_x={error_x:.2f} rudder={rudder:.1f}°")

    def _estado_acercarse(self):
        """
        Objeto centrado. Avanza hacia él corrigiendo el timón continuamente.
        Si el objeto crece mucho (área grande) → RECOGER.
        Si pierde el objeto → APUNTAR.
        """
        if not self.target_detected:
            self._cambiar_estado(Estado.APUNTAR)
            return

        # Si el objeto ya es muy grande = estamos cerca
        if self.target_area >= self.collect_area:
            self._cambiar_estado(Estado.RECOGER)
            return

        # Visual servoing: corregir timón continuamente
        rudder = self._rudder_desde_camara()
        self._set_rudder(rudder)

        # Velocidad inversamente proporcional al error lateral
        # (más despacio si hay que corregir mucho)
        factor = 1.0 - abs(self.target_x) * 0.4
        spd = self.approach_spd * factor
        self._set_motors(spd, spd)

        # Evasión suave si hay obstáculo cercano pero no crítico
        if self.obstacle_dist < self.obs_evade:
            reduccion = 1.0 - (
                (self.obs_evade - self.obstacle_dist) / self.obs_evade)
            spd *= max(0.1, reduccion)
            self._set_motors(spd, spd)

        self.get_logger().debug(
            f"ACERCARSE area={self.target_area:.3f} "
            f"x={self.target_x:.2f} spd={spd:.2f}")

    def _estado_recoger(self):
        """
        Objeto muy cerca. Avanza despacio para que entre en la banda.
        La banda ya está activa permanentemente.
        Después de collect_time segundos → volver a BUSCAR.
        """
        rudder = self._rudder_desde_camara()
        self._set_rudder(rudder)
        self._set_motors(self.collect_spd, self.collect_spd)

        if self._tiempo_en_estado() >= self.collect_time:
            self.get_logger().info(
                f"Recoleccion completada ({self.collect_time}s) → BUSCAR")
            self._stop()
            self._cambiar_estado(Estado.BUSCAR)

    def _estado_evadir(self):
        """
        Obstáculo muy cerca. Para motores y retrocede un poco,
        luego gira para esquivar y vuelve al estado anterior.
        """
        t = self._tiempo_en_estado()

        if t < 0.8:
            # Fase 1: retroceder
            self._set_motors(-0.3, -0.3)
            self._set_rudder(0.0)
        elif t < 1.6:
            # Fase 2: girar para esquivar (siempre a la derecha)
            self._set_motors(0.3, -0.3)
            self._set_rudder(30.0)
        else:
            # Fase 3: obstáculo esquivado, volver al estado anterior
            self._stop()
            estado_retorno = (
                self.estado_previo
                if self.estado_previo else Estado.BUSCAR)
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
    # Parar todo al salir
    node._stop()
    node._set_conveyor(False)
    node.destroy_node()
    rclpy.shutdown()
