import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import math

class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu_data', 10)

        self.bus = smbus.SMBus(1)
        self.address = 0x68
        self.bus.write_byte_data(self.address, 0x6B, 0)  # Wake up MPU6050

        self.timer = self.create_timer(0.5, self.publish_imu)

    def read_word_2c(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg+1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def publish_imu(self):
        imu_msg = Imu()
        accel_x = self.read_word_2c(0x3B) / 16384.0
        accel_y = self.read_word_2c(0x3D) / 16384.0
        accel_z = self.read_word_2c(0x3F) / 16384.0

        # Simplificado: solo guardamos aceleraciones
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        self.publisher_.publish(imu_msg)
        self.get_logger().info(f"IMU accel: {accel_x}, {accel_y}, {accel_z}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
