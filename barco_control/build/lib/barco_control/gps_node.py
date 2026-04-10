import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
import threading
import time

class GPSNode(Node):
    """
    Lee sentencias NMEA del módulo NEO-M8N por serial.
    Convierte correctamente el formato NMEA (DDMM.MMMMM) a grados decimales.
    Publica en el topic 'gps_data' solo cuando hay fix válido.
    """

    def __init__(self):
        super().__init__('gps_node')

        self.publisher_ = self.create_publisher(NavSatFix, 'gps_data', 10)

        # NEO-M8N por defecto usa 9600 baud en /dev/ttyAMA0
        # Si lo conectas por USB usa /dev/ttyUSB1 (USB0 es el ESP32)
        self.serial_port = '/dev/ttyAMA0'
        self.baudrate    = 9600
        self.ser         = None
        self._stop       = False

        try:
            self.ser = serial.Serial(
                self.serial_port, self.baudrate, timeout=2.0)
            self.get_logger().info(
                f"GPS serial abierto: {self.serial_port} @ {self.baudrate}")
            self.read_thread = threading.Thread(
                target=self.read_loop, daemon=True)
            self.read_thread.start()
        except Exception as e:
            self.get_logger().error(f"No se pudo abrir GPS serial: {e}")

    # ────────────────────────────────────────────
    # Conversión NMEA → grados decimales
    # ────────────────────────────────────────────

    def nmea_to_decimal(self, value: str, direction: str) -> float:
        """
        Convierte DDMM.MMMMM + N/S/E/W a grados decimales.
        Ejemplo: '1955.1234', 'N' → 19 + 55.1234/60 = 19.9187...
        """
        if not value or value.strip() == '':
            return 0.0
        try:
            raw = float(value)
            degrees = int(raw / 100)
            minutes = raw - degrees * 100
            decimal = degrees + minutes / 60.0
            if direction in ('S', 'W'):
                decimal *= -1
            return decimal
        except ValueError:
            return 0.0

    # ────────────────────────────────────────────
    # Parseo de sentencias NMEA
    # ────────────────────────────────────────────

    def parse_gga(self, parts: list):
        """
        $GPGGA o $GNGGA — posicion, calidad de fix, satelites, altitud.
        Indice: 0=tipo, 1=UTC, 2=lat, 3=N/S, 4=lon, 5=E/W,
                6=fix_quality (0=sin fix), 7=sats, 8=hdop,
                9=altitud, 10=M, ...
        """
        if len(parts) < 10:
            return

        fix_quality = int(parts[6]) if parts[6].isdigit() else 0
        if fix_quality == 0:
            self.get_logger().warn("GPS sin fix — esperando señal...")
            return

        lat = self.nmea_to_decimal(parts[2], parts[3])
        lon = self.nmea_to_decimal(parts[4], parts[5])

        try:
            alt = float(parts[9]) if parts[9] else 0.0
        except ValueError:
            alt = 0.0

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'

        msg.status.status  = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude  = lat
        msg.longitude = lon
        msg.altitude  = alt

        # Covarianza desconocida
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"GPS fix: lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}m "
            f"sats={parts[7]}")

    def parse_rmc(self, parts: list):
        """
        $GPRMC o $GNRMC — posicion y velocidad.
        Solo usamos como respaldo si GGA no está disponible.
        Indice: 0=tipo, 1=UTC, 2=status(A=activo), 3=lat, 4=N/S,
                5=lon, 6=E/W, 7=velocidad_nudos, ...
        """
        if len(parts) < 7:
            return
        if parts[2] != 'A':   # A = datos válidos, V = advertencia
            return

        lat = self.nmea_to_decimal(parts[3], parts[4])
        lon = self.nmea_to_decimal(parts[5], parts[6])

        msg = NavSatFix()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status.status   = NavSatStatus.STATUS_FIX
        msg.latitude        = lat
        msg.longitude       = lon
        msg.altitude        = 0.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.publisher_.publish(msg)
        self.get_logger().debug(f"GPS RMC: lat={lat:.6f} lon={lon:.6f}")

    # ────────────────────────────────────────────
    # Verificar checksum NMEA
    # ────────────────────────────────────────────

    def valid_checksum(self, sentence: str) -> bool:
        try:
            if '*' not in sentence:
                return True   # sin checksum, aceptar
            data, checksum = sentence.split('*', 1)
            data = data.lstrip('$')
            calculated = 0
            for c in data:
                calculated ^= ord(c)
            return calculated == int(checksum[:2], 16)
        except Exception:
            return False

    # ────────────────────────────────────────────
    # Loop de lectura serial
    # ────────────────────────────────────────────

    def read_loop(self):
        while not self._stop:
            try:
                raw  = self.ser.readline()
                line = raw.decode('ascii', errors='ignore').strip()

                if not line.startswith('$'):
                    continue

                if not self.valid_checksum(line):
                    self.get_logger().debug(f"Checksum inválido: {line}")
                    continue

                # Quitar checksum para parsear
                sentence = line.split('*')[0]
                parts    = sentence.split(',')
                msg_type = parts[0]

                if msg_type in ('$GPGGA', '$GNGGA'):
                    self.parse_gga(parts)
                elif msg_type in ('$GPRMC', '$GNRMC'):
                    self.parse_rmc(parts)

            except Exception as e:
                if not self._stop:
                    self.get_logger().error(f"Error leyendo GPS: {e}")
                time.sleep(0.5)

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
    node = GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
