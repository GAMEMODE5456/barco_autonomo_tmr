import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import serial
import threading
import time

class ESPBridgeNode(Node):

    def __init__(self):
        super().__init__('esp_bridge_node')
        self.dist_pub = self.create_publisher(Float32, 'obstacle_distance', 10)
        self.conv_pub = self.create_publisher(Bool, 'conveyor_status', 10)

        # Ajusta el puerto según tu sistema
        self.serial_port = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.timeout = 1.0

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=self.timeout)
            self.get_logger().info(f"Serial abierto en {self.serial_port} @ {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"No se pudo abrir serial {self.serial_port}: {e}")
            self.ser = None

        self._stop = False
        if self.ser:
            self.thread = threading.Thread(target=self.read_loop, daemon=True)
            self.thread.start()

        # Watchdog: si no llegan DIST en X segundos, publicar distancia grande segura
        self.last_dist_time = time.time()
        self.watchdog_timer = self.create_timer(1.0, self.watchdog_check)

    def read_loop(self):
        while not self._stop:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                # parsear
                if line.startswith("DIST:"):
                    try:
                        val = float(line.split(":",1)[1])
                        msg = Float32()
                        msg.data = val
                        self.dist_pub.publish(msg)
                        self.get_logger().debug(f"DIST publicado: {val}")
                        self.last_dist_time = time.time()
                    except ValueError:
                        self.get_logger().warn(f"Valor DIST inválido: {line}")
                elif line.startswith("CONV:"):
                    state = line.split(":",1)[1].upper() == "ON"
                    msg = Bool()
                    msg.data = state
                    self.conv_pub.publish(msg)
                    self.get_logger().info(f"CONV publicado: {'ON' if state else 'OFF'}")
                elif line.startswith("HB:"):
                    # opcional: heartbeat
                    pass
                else:
                    self.get_logger().debug(f"Linea serial no reconocida: {line}")
            except Exception as e:
                self.get_logger().error(f"Error leyendo serial: {e}")
                time.sleep(0.5)

    def watchdog_check(self):
        # Si no hay lectura reciente, publicar distancia grande para seguridad
        if time.time() - self.last_dist_time > 2.0:
            msg = Float32()
            msg.data = 9999.0
            self.dist_pub.publish(msg)
            self.get_logger().warn("No se reciben DIST desde ESP32, publicando distancia segura 9999")

    def destroy_node(self):
        self._stop = True
        if self.ser:
            try:
                self.ser.close()
            except:
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
