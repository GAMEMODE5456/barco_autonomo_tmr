import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial

class GPSNode(Node):

    def __init__(self):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_data', 10)

        # Configura el puerto serial según tu módulo GPS
        self.gps = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1)
        self.timer = self.create_timer(1.0, self.publish_position)

    def publish_position(self):
        line = self.gps.readline().decode('utf-8', errors='ignore')
        if line.startswith('$GPGGA'):  # Ejemplo con sentencia NMEA GGA
            data = line.split(',')
            msg = NavSatFix()
            msg.latitude = float(data[2])  # simplificado, requiere conversión NMEA
            msg.longitude = float(data[4])
            msg.altitude = float(data[9])
            self.publisher_.publish(msg)
            self.get_logger().info(f"GPS: {msg.latitude}, {msg.longitude}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
