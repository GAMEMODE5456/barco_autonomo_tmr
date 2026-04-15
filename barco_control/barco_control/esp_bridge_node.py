import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool
import serial
import threading
import time

class ESPBridgeNode(Node):
    """
    Unico nodo que posee el puerto serial hacia el ESP32.
    Recibe topics de ROS2 y los traduce a comandos seriales.
    Recibe datos seriales del ESP32 y los publica como topics.

    Protocolo serial (Raspberry → ESP32):
      MOT:0.50\\n             velocidad motor unico [-1.0 .. 1.0]
      SRV:30.0\\n             angulo servo timon [-60 .. 60]
      CONV:ON\\n / CONV:OFF\\n banda transportadora

    Protocolo serial (ESP32 → Raspberry):
      DIST:32.5\\n            distancia ultrasonico cm
      CONV:ON\\n / CONV:OFF\\n confirmacion estado banda
      HB:OK\\n                heartbeat opcional
    """

    def __init__(self):
        super().__init__('esp_bridge_node')

        # ── Subscripciones desde otros nodos ROS2 ──
        self.create_subscription(
            Float32MultiArray, 'motor_speeds',
            self.cb_motor_speeds, 10)

        self.create_subscription(
            Float32, 'rudder_angle',
            self.cb_rudder, 10)

        self.create_subscription(
            Bool, 'conveyor_power',
            self.cb_conveyor, 10)

        # ── Publishers hacia otros nodos ROS2 ──
        self.dist_pub = self.create_publisher(Float32, 'obstacle_distance_esp', 10)
        self.conv_pub = self.create_publisher(Bool,    'conveyor_status',       10)

        # ── Puerto serial ──
        self.serial_port = '/dev/ttyACM0'
        self.baudrate    = 115200
        self.ser         = None
        self._stop       = False
        self._lock       = threading.Lock()   # protege escrituras concurrentes

        try:
            self.ser = serial.Serial(
                self.serial_port, self.baudrate, timeout=1.0)
            self.get_logger().info(
                f"Serial abierto: {self.serial_port} @ {self.baudrate}")
            self.read_thread = threading.Thread(
                target=self.read_loop, daemon=True)
            self.read_thread.start()
        except Exception as e:
            self.get_logger().error(f"No se pudo abrir serial: {e}")

        # ── Watchdog: si no llegan DIST en 2s publica distancia segura ──
        self.last_dist_time = time.time()
        self.create_timer(1.0, self.watchdog_check)

    # ────────────────────────────────────────────
    # Callbacks de subscripciones
    # ────────────────────────────────────────────

    def cb_motor_speeds(self, msg: Float32MultiArray):
        """Recibe [velocidad] de motor_node y lo envía al ESP32."""
        vals = msg.data
        if len(vals) == 0:
            return
        vel = float(vals[0])
        vel = max(-1.0, min(1.0, vel))
        self.send(f"MOT:{vel:.2f}")

    def cb_rudder(self, msg: Float32):
        """Rango ampliado a ±60°."""
        angle = max(-60.0, min(60.0, msg.data))
        self.send(f"SRV:{angle:.1f}")

    def cb_conveyor(self, msg: Bool):
        self.send("CONV:ON" if msg.data else "CONV:OFF")

    # ────────────────────────────────────────────
    # Envío serial (thread-safe)
    # ────────────────────────────────────────────

    def send(self, line: str):
        if not self.ser:
            return
        with self._lock:
            try:
                self.ser.write((line + '\n').encode('utf-8'))
                self.get_logger().debug(f"TX → ESP32: {line}")
            except Exception as e:
                self.get_logger().error(f"Error TX serial: {e}")

    # ────────────────────────────────────────────
    # Lectura serial en hilo separado
    # ────────────────────────────────────────────

    def read_loop(self):
        while not self._stop:
            try:
                raw  = self.ser.readline()
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                self.get_logger().debug(f"RX ← ESP32: {line}")

                if line.startswith("DIST:"):
                    try:
                        val = float(line.split(":", 1)[1])
                        msg = Float32()
                        msg.data = val
                        self.dist_pub.publish(msg)
                        self.last_dist_time = time.time()
                    except ValueError:
                        self.get_logger().warn(f"DIST invalido: {line}")

                elif line.startswith("CONV:"):
                    state = line.split(":", 1)[1].upper() == "ON"
                    msg = Bool()
                    msg.data = state
                    self.conv_pub.publish(msg)

                elif line.startswith("HB:"):
                    pass  # heartbeat, ignorar

                else:
                    self.get_logger().debug(f"Linea no reconocida: {line}")

            except Exception as e:
                if not self._stop:
                    self.get_logger().error(f"Error RX serial: {e}")
                time.sleep(0.5)

    # ────────────────────────────────────────────
    # Watchdog
    # ────────────────────────────────────────────

    def watchdog_check(self):
        if time.time() - self.last_dist_time > 2.0:
            msg = Float32()
            msg.data = 9999.0
            self.dist_pub.publish(msg)
            self.get_logger().warn(
                "Sin DIST desde ESP32 — publicando distancia segura 9999")

    def destroy_node(self):
        self._stop = True
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESPBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
