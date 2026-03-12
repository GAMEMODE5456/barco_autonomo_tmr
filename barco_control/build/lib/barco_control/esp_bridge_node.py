import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import serial

class ESPBridgeNode(Node):

    def __init__(self):
        super().__init__('esp_bridge_node')

        # Publicadores
        self.dist_pub = self.create_publisher(Float32, 'obstacle_distance', 10)
        self.conv_pub = self.create_publisher(Bool, 'conveyor_status', 10)

        # Configura el puerto serial (ajusta según tu ESP32)
        self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)

        self.timer = self.create_timer(0.5, self.read_data)

    def read_data(self):
        line = self.ser.readline().decode('utf-8').strip()
        if line.startswith("DIST:"):
            dist = float(line.split(":")[1])
            msg = Float32()
            msg.data = dist
            self.dist_pub.publish(msg)
            self.get_logger().info(f"Distancia recibida: {dist} cm")

        elif line.startswith("CONV:"):
            status = line.split(":")[1] == "ON"
            msg = Bool()
            msg.data = status
            self.conv_pub.publish(msg)
            self.get_logger().info(f"Banda: {'ACTIVA' if status else 'APAGADA'}")

def main(args=None):
    rclpy.init(args=args)
    node = ESPBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
