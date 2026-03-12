import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, Imu

class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        self.create_subscription(NavSatFix, 'gps_data', self.gps_callback, 10)
        self.create_subscription(Imu, 'imu_data', self.imu_callback, 10)

        self.rudder_pub = self.create_publisher(Float32, 'rudder_angle', 10)
        self.motor_pub = self.create_publisher(Float32, 'motor_speed', 10)

        self.current_lat = None
        self.current_lon = None
        self.current_yaw = None

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.get_logger().info(f"GPS recibido: {self.current_lat}, {self.current_lon}")

    def imu_callback(self, msg):
        self.current_yaw = msg.orientation.z  # simplificado
        self.get_logger().info(f"IMU yaw: {self.current_yaw}")

        if self.current_lat and self.current_lon:
            rudder = Float32()
            rudder.data = 0.0  # lógica de navegación real aquí
            self.rudder_pub.publish(rudder)

            motor = Float32()
            motor.data = 0.5  # velocidad fija por ahora
            self.motor_pub.publish(motor)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
