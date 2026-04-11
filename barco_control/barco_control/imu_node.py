import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import smbus2
import math
import time

class IMUNode(Node):
    """
    Lee el MPU6050 por I2C y publica datos completos.

    Publica:
      'imu_data'   (Imu)     — aceleración, velocidad angular, orientación estimada
      'imu_yaw'    (Float32) — yaw en grados (útil para navigation_node)

    Correcciones respecto al original:
      - Lee giroscopio además de acelerómetro
      - Calcula yaw por integración del giroscopio Z
      - Aplica filtro complementario para roll y pitch estables
      - Calibra offsets al arrancar (bias del sensor)
      - Maneja errores I2C sin crashear
    """

    MPU6050_ADDR   = 0x68
    PWR_MGMT_1     = 0x6B
    ACCEL_XOUT_H   = 0x3B
    GYRO_XOUT_H    = 0x43

    # Escalas por defecto (rango ±2g accel, ±250°/s gyro)
    ACCEL_SCALE    = 16384.0   # LSB/g
    GYRO_SCALE     = 131.0     # LSB/°/s

    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('i2c_bus',          1)
        self.declare_parameter('publish_rate_hz',  20.0)
        self.declare_parameter('alpha',            0.98)   # filtro complementario
        self.declare_parameter('calibration_secs', 2.0)    # segundos de calibración

        bus_num      = self.get_parameter('i2c_bus').value
        rate_hz      = self.get_parameter('publish_rate_hz').value
        self.alpha   = self.get_parameter('alpha').value   # peso giroscopio
        cal_secs     = self.get_parameter('calibration_secs').value

        # Publishers
        self.imu_pub = self.create_publisher(Imu,     'imu_data', 10)
        self.yaw_pub = self.create_publisher(Float32, 'imu_yaw',  10)

        # I2C
        try:
            self.bus = smbus2.SMBus(bus_num)
            self.bus.write_byte_data(
                self.MPU6050_ADDR, self.PWR_MGMT_1, 0)  # wake up
            self.get_logger().info(
                f"MPU6050 iniciado en bus I2C {bus_num}")
        except Exception as e:
            self.get_logger().error(f"Error iniciando MPU6050: {e}")
            self.bus = None

        # Estado del filtro
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0
        self.last_time = time.time()

        # Offsets de calibración (bias)
        self.gyro_offset_x = 0.0
        self.gyro_offset_y = 0.0
        self.gyro_offset_z = 0.0

        # Calibrar al arrancar
        if self.bus:
            self._calibrar(cal_secs)

        dt = 1.0 / rate_hz
        self.create_timer(dt, self.publish_imu)

    # ─────────────────────────────────────────
    # Lectura raw I2C
    # ─────────────────────────────────────────

    def _read_word_2c(self, reg: int) -> int:
        try:
            high = self.bus.read_byte_data(self.MPU6050_ADDR, reg)
            low  = self.bus.read_byte_data(self.MPU6050_ADDR, reg + 1)
            val  = (high << 8) | low
            return -((65535 - val) + 1) if val >= 0x8000 else val
        except Exception as e:
            self.get_logger().warn(f"Error leyendo I2C reg {reg}: {e}")
            return 0

    def _read_all(self):
        """Lee acelerómetro y giroscopio en una sola ráfaga."""
        try:
            data = self.bus.read_i2c_block_data(
                self.MPU6050_ADDR, self.ACCEL_XOUT_H, 14)

            def to_signed(h, l):
                v = (h << 8) | l
                return -((65535 - v) + 1) if v >= 0x8000 else v

            ax = to_signed(data[0],  data[1])  / self.ACCEL_SCALE
            ay = to_signed(data[2],  data[3])  / self.ACCEL_SCALE
            az = to_signed(data[4],  data[5])  / self.ACCEL_SCALE
            gx = to_signed(data[8],  data[9])  / self.GYRO_SCALE
            gy = to_signed(data[10], data[11]) / self.GYRO_SCALE
            gz = to_signed(data[12], data[13]) / self.GYRO_SCALE

            return ax, ay, az, gx, gy, gz
        except Exception as e:
            self.get_logger().warn(f"Error lectura bloque I2C: {e}")
            return 0.0, 0.0, 1.0, 0.0, 0.0, 0.0

    # ─────────────────────────────────────────
    # Calibración de bias del giroscopio
    # ─────────────────────────────────────────

    def _calibrar(self, secs: float):
        self.get_logger().info(
            f"Calibrando MPU6050 ({secs}s) — mantener barco quieto...")
        muestras = 0
        sx = sy = sz = 0.0
        t_ini = time.time()

        while time.time() - t_ini < secs:
            _, _, _, gx, gy, gz = self._read_all()
            sx += gx
            sy += gy
            sz += gz
            muestras += 1
            time.sleep(0.01)

        if muestras > 0:
            self.gyro_offset_x = sx / muestras
            self.gyro_offset_y = sy / muestras
            self.gyro_offset_z = sz / muestras

        self.get_logger().info(
            f"Calibración OK — offsets gyro: "
            f"x={self.gyro_offset_x:.3f} "
            f"y={self.gyro_offset_y:.3f} "
            f"z={self.gyro_offset_z:.3f}°/s")

    # ─────────────────────────────────────────
    # Filtro complementario
    # ─────────────────────────────────────────

    def _update_filter(self, ax, ay, az, gx, gy, gz, dt):
        """
        Combina acelerómetro (ángulo absoluto) con giroscopio (velocidad angular).
        alpha=0.98 → 98% giroscopio + 2% acelerómetro en cada paso.
        Yaw solo por giroscopio (el acelerómetro no da yaw).
        """
        # Ángulo del acelerómetro (pitch y roll)
        accel_roll  = math.atan2(ay, az)
        accel_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

        # Corregir bias
        gx -= self.gyro_offset_x
        gy -= self.gyro_offset_y
        gz -= self.gyro_offset_z

        # Integrar giroscopio
        gyro_roll  = self.roll  + math.radians(gx) * dt
        gyro_pitch = self.pitch + math.radians(gy) * dt
        self.yaw  += math.radians(gz) * dt   # yaw solo integración

        # Filtro complementario para roll y pitch
        self.roll  = self.alpha * gyro_roll  + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch

        # Normalizar yaw a [-π, π]
        self.yaw = math.atan2(
            math.sin(self.yaw), math.cos(self.yaw))

        return gx, gy, gz

    # ─────────────────────────────────────────
    # Publicación
    # ─────────────────────────────────────────

    def publish_imu(self):
        if not self.bus:
            return

        now = time.time()
        dt  = now - self.last_time
        self.last_time = now

        ax, ay, az, gx, gy, gz = self._read_all()
        gx, gy, gz = self._update_filter(ax, ay, az, gx, gy, gz, dt)

        # ── Imu msg ──
        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu'

        msg.linear_acceleration.x = ax * 9.81
        msg.linear_acceleration.y = ay * 9.81
        msg.linear_acceleration.z = az * 9.81

        msg.angular_velocity.x = math.radians(gx)
        msg.angular_velocity.y = math.radians(gy)
        msg.angular_velocity.z = math.radians(gz)

        # Orientación estimada (roll, pitch, yaw en radianes)
        msg.orientation.x = self.roll
        msg.orientation.y = self.pitch
        msg.orientation.z = self.yaw   # navigation_node lee esto
        msg.orientation.w = 1.0

        # Covarianzas desconocidas
        msg.orientation_covariance[0]          = -1.0
        msg.angular_velocity_covariance[0]     = -1.0
        msg.linear_acceleration_covariance[0]  = -1.0

        self.imu_pub.publish(msg)

        # ── Yaw en grados (más fácil de usar) ──
        yaw_deg = Float32()
        yaw_deg.data = math.degrees(self.yaw)
        self.yaw_pub.publish(yaw_deg)

        self.get_logger().debug(
            f"IMU roll={math.degrees(self.roll):.1f}° "
            f"pitch={math.degrees(self.pitch):.1f}° "
            f"yaw={math.degrees(self.yaw):.1f}°")

    def destroy_node(self):
        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
