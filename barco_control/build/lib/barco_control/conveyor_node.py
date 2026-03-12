import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class ConveyorNode(Node):

    def __init__(self):
        super().__init__('conveyor_node')

        # Suscripción a un tópico que indica si la banda debe estar encendida
        self.subscription = self.create_subscription(
            Bool,
            'conveyor_power',
            self.listener_callback,
            10
        )

        self.get_logger().info("Nodo de banda transportadora inicializado")

    def listener_callback(self, msg):
        if msg.data:
            self.activate_conveyor()
        else:
            self.deactivate_conveyor()

    def activate_conveyor(self):
        # Aquí iría el código para energizar el motor (GPIO, PCA9685, etc.)
        self.get_logger().info("Banda transportadora ACTIVADA")

    def deactivate_conveyor(self):
        # Aquí iría el código para cortar energía al motor
        self.get_logger().info("Banda transportadora DESACTIVADA")

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
