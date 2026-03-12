import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import threading
import time

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_speeds',
            self.callback_motor_speeds,
            10
        )
        self.serial_port = '/dev/ttyUSB0'  # ajustar
        self.baudrate = 115200
        self.ser = None
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f"Serial abierto en {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"No se pudo abrir serial: {e}")
            self.ser = None

        self.last_cmd_time = time.time()
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_check)

    def callback_motor_speeds(self, msg: Float32MultiArray):
        # Esperamos 1 o 2 valores. Si 1 valor -> aplica a ambos motores.
        vals = msg.data
        if len(vals) == 0:
            return
        if len(vals) == 1:
            left = right = float(vals[0])
        else:
            left = float(vals[0])
            right = float(vals[1])

        # Limitar rango [-1.0, 1.0]
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        self.send_to_esp(left, right)
        self.last_cmd_time = time.time()
        self.get_logger().info(f"Cmd motores L:{left:.2f} R:{right:.2f}")

    def send_to_esp(self, left, right):
        if not self.ser:
            return
        # Formato simple: MOT:L:0.50,R:0.50\n  (valores -1.00 .. 1.00)
        line = f"MOT:L:{left:.2f},R:{right:.2f}\n"
        try:
            self.ser.write(line.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Error enviando serial: {e}")

    def watchdog_check(self):
        # Si no hay comandos recientes, enviar stop (0,0)
        if time.time() - self.last_cmd_time > 1.0:
            self.send_to_esp(0.0, 0.0)

    def destroy_node(self):
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
