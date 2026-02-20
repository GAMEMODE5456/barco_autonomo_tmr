import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')
        self.publisher_ = self.create_publisher(Float32, 'motor_speed', 10)
        self.timer = self.create_timer(1.0, self.publish_speed)
        self.speed = 0.5  # 50% potencia inicial

    def publish_speed(self):
        msg = Float32()
        msg.data = self.speed
        self.publisher_.publish(msg)
        self.get_logger().info(f"Velocidad enviada: {self.speed}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
