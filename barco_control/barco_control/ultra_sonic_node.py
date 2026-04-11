import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time
import threading

class UltrasonicNode(Node):
    """
    Sensor ultrasónico HC-SR04 conectado directo a la Raspberry Pi.

    Correcciones respecto al original:
      - Topic renombrado a 'obstacle_distance_sonar' para no
        pisarse con esp_bridge_node que publica en 'obstacle_distance'
      - Publicador adicional en 'obstacle_distance' como fusión
        (toma el menor entre sonar y ESP32)
      - Timeout de seguridad en pulseIn (evita colgarse si ECHO no baja)
      - Medición en hilo separado (no bloquea el executor de ROS2)
      - Filtro de mediana para descartar lecturas espurias del agua

    Pines (GPIO BCM):
      TRIG = GPIO 12
      ECHO = GPIO 13   ← OJO: HC-SR04 da 5V en ECHO
                          usar divisor resistivo 1kΩ/2kΩ o
                          módulo con lógica 3.3V antes de conectar a Raspi
    """

    def __init__(self):
        super().__init__('ultra_sonic_node')

        self.declare_parameter('trig_pin',       12)
        self.declare_parameter('echo_pin',       13)
        self.declare_parameter('measure_rate_hz', 5.0)
        self.declare_parameter('max_distance_cm', 400.0)
        self.declare_parameter('min_distance_cm',   2.0)
        self.declare_parameter('median_window',    5)     # muestras para mediana

        self.TRIG      = self.get_parameter('trig_pin').value
        self.ECHO      = self.get_parameter('echo_pin').value
        rate_hz        = self.get_parameter('measure_rate_hz').value
        self.max_dist  = self.get_parameter('max_distance_cm').value
        self.min_dist  = self.get_parameter('min_distance_cm').value
        self.win_size  = self.get_parameter('median_window').value

        # Buffer para filtro de mediana
        self.buffer    = []
        self._lock     = threading.Lock()
        self._stop     = False

        # Publisher propio (solo sonar)
        self.sonar_pub = self.create_publisher(
            Float32, 'obstacle_distance_sonar', 10)

        # Publisher fusionado (navigation_node suscribe a este)
        # Se actualiza cuando llega dato del ESP32 también
        self.fused_pub = self.create_publisher(
            Float32, 'obstacle_distance', 10)

        # Suscribirse a la distancia del ESP32 para fusión
        self.esp_dist  = 9999.0
        self.create_subscription(
            Float32, 'obstacle_distance_esp',
            self.cb_esp_dist, 10)

        # GPIO
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.TRIG, GPIO.OUT)
            GPIO.setup(self.ECHO, GPIO.IN)
            GPIO.output(self.TRIG, False)
            time.sleep(0.1)   # dejar estabilizar sensor
            self.gpio_ok = True
            self.get_logger().info(
                f"HC-SR04 GPIO OK — TRIG={self.TRIG} ECHO={self.ECHO}")
        except Exception as e:
            self.get_logger().error(f"Error GPIO: {e}")
            self.gpio_ok = False

        # Hilo de medición (no bloquea ROS2)
        if self.gpio_ok:
            self._thread = threading.Thread(
                target=self._measure_loop,
                args=(1.0 / rate_hz,),
                daemon=True)
            self._thread.start()

        # Timer para publicar a ROS2 (desde el hilo principal)
        self.last_distance = self.max_dist
        self.create_timer(1.0 / rate_hz, self._publish)

    # ─────────────────────────────────────────
    # Medición hardware
    # ─────────────────────────────────────────

    def _medir(self) -> float:
        """
        Dispara pulso ultrasónico y mide el eco.
        Retorna distancia en cm o max_dist si hay timeout.
        """
        try:
            # Pulso trigger 10µs
            GPIO.output(self.TRIG, True)
            time.sleep(0.00001)
            GPIO.output(self.TRIG, False)

            timeout = time.time() + 0.03   # 30ms máximo

            # Esperar inicio del eco
            start = time.time()
            while GPIO.input(self.ECHO) == 0:
                start = time.time()
                if time.time() > timeout:
                    return self.max_dist

            # Esperar fin del eco
            end = time.time()
            while GPIO.input(self.ECHO) == 1:
                end = time.time()
                if time.time() > timeout:
                    return self.max_dist

            duracion = end - start
            distancia = (duracion * 34300.0) / 2.0

            # Filtrar lecturas fuera de rango
            if distancia < self.min_dist or distancia > self.max_dist:
                return self.max_dist

            return distancia

        except Exception as e:
            self.get_logger().warn(f"Error midiendo HC-SR04: {e}")
            return self.max_dist

    def _mediana(self, valores: list) -> float:
        s = sorted(valores)
        n = len(s)
        mid = n // 2
        return s[mid] if n % 2 else (s[mid - 1] + s[mid]) / 2.0

    def _measure_loop(self, interval: float):
        """Hilo de medición continua."""
        while not self._stop:
            dist = self._medir()
            with self._lock:
                self.buffer.append(dist)
                if len(self.buffer) > self.win_size:
                    self.buffer.pop(0)
                if self.buffer:
                    self.last_distance = self._mediana(self.buffer)
            time.sleep(interval)

    # ─────────────────────────────────────────
    # Fusión con distancia del ESP32
    # ─────────────────────────────────────────

    def cb_esp_dist(self, msg: Float32):
        self.esp_dist = msg.data

    # ─────────────────────────────────────────
    # Publicación
    # ─────────────────────────────────────────

    def _publish(self):
        with self._lock:
            dist_sonar = self.last_distance

        # Publicar sonar propio
        msg_sonar = Float32()
        msg_sonar.data = dist_sonar
        self.sonar_pub.publish(msg_sonar)

        # Fusión: tomar el mínimo entre sonar y ESP32
        # (el obstáculo más cercano gana)
        dist_fused = min(dist_sonar, self.esp_dist)
        msg_fused = Float32()
        msg_fused.data = dist_fused
        self.fused_pub.publish(msg_fused)

        self.get_logger().debug(
            f"Sonar={dist_sonar:.1f}cm "
            f"ESP32={self.esp_dist:.1f}cm "
            f"Fused={dist_fused:.1f}cm")

    # ─────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────

    def destroy_node(self):
        self._stop = True
        if self.gpio_ok:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
