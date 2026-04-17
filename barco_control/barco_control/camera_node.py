import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

class CameraNode(Node):
    """
    Detecta tres tipos de objetos:
      Verde  → sargaso verde  → recoger  → 'target_detected' / 'target_position'
      Cafe   → sargaso cafe   → recoger  → 'target2_detected' / 'target2_position'
      Boya   → boya roja      → limite   → 'boya_detected' / 'boya_position'

    Prioridad de publicación de target:
      Si hay verde Y café visibles, publica el más grande como target principal.
      Las boyas siempre se publican en su propio topic.
    """

    def __init__(self):
        super().__init__('camera_node')

        # ── Parámetros ──────────────────────────────────
        self.declare_parameter('camera_index',       0)
        self.declare_parameter('frame_width',        640)
        self.declare_parameter('frame_height',       480)
        self.declare_parameter('fps',                30)
        self.declare_parameter('min_area',           500.0)
        self.declare_parameter('publish_image',      False)

        # Sargaso verde
        self.declare_parameter('hsv_lower_verde', [35, 60, 60])
        self.declare_parameter('hsv_upper_verde', [75, 255, 255])

        # Sargaso café/morado
        self.declare_parameter('hsv_lower_cafe',  [120, 40, 30])
        self.declare_parameter('hsv_upper_cafe',  [170, 180, 150])

        # Boyas rojas/naranjas
        self.declare_parameter('hsv_lower_boya',  [0, 150, 150])
        self.declare_parameter('hsv_upper_boya',  [15, 255, 255])
        self.declare_parameter('hsv_lower_boya2', [160, 150, 150])
        self.declare_parameter('hsv_upper_boya2', [180, 255, 255])

        self.cam_idx   = self.get_parameter('camera_index').value
        self.frame_w   = self.get_parameter('frame_width').value
        self.frame_h   = self.get_parameter('frame_height').value
        self.fps       = self.get_parameter('fps').value
        self.min_area  = self.get_parameter('min_area').value
        self.pub_image = self.get_parameter('publish_image').value

        self.lower_verde  = np.array(self.get_parameter('hsv_lower_verde').value)
        self.upper_verde  = np.array(self.get_parameter('hsv_upper_verde').value)
        self.lower_cafe   = np.array(self.get_parameter('hsv_lower_cafe').value)
        self.upper_cafe   = np.array(self.get_parameter('hsv_upper_cafe').value)
        self.lower_boya   = np.array(self.get_parameter('hsv_lower_boya').value)
        self.upper_boya   = np.array(self.get_parameter('hsv_upper_boya').value)
        self.lower_boya2  = np.array(self.get_parameter('hsv_lower_boya2').value)
        self.upper_boya2  = np.array(self.get_parameter('hsv_upper_boya2').value)

        # ── Publishers ──────────────────────────────────
        # Sargaso (verde o café — el más grande)
        self.target_detected_pub  = self.create_publisher(Bool,              'target_detected',  10)
        self.target_position_pub  = self.create_publisher(Float32MultiArray, 'target_position',  10)

        # Boyas — límite de zona
        self.boya_detected_pub    = self.create_publisher(Bool,              'boya_detected',    10)
        self.boya_position_pub    = self.create_publisher(Float32MultiArray, 'boya_position',    10)

        if self.pub_image:
            self.image_pub = self.create_publisher(Image, 'camera_image', 10)

        self.bridge = CvBridge()
        self.cap    = None
        self._stop  = False
        self._open_camera()

        # Contadores para filtro de pérdida
        self.frames_sin_target = 0
        self.frames_sin_boya   = 0
        self.FRAMES_PERDIDA    = 10

        self._thread = threading.Thread(target=self.capture_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"CameraNode listo — cam={self.cam_idx} {self.frame_w}x{self.frame_h}")
        self.get_logger().info(
            f"Verde lower={self.lower_verde} upper={self.upper_verde}")
        self.get_logger().info(
            f"Cafe  lower={self.lower_cafe}  upper={self.upper_cafe}")
        self.get_logger().info(
            f"Boya  lower={self.lower_boya}  upper={self.upper_boya}")

    # ─────────────────────────────────────────
    # Abrir cámara
    # ─────────────────────────────────────────

    def _open_camera(self):
        self.cap = cv2.VideoCapture(self.cam_idx)
        if not self.cap.isOpened():
            self.get_logger().error(f"No se pudo abrir la cámara {self.cam_idx}")
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.frame_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
        self.cap.set(cv2.CAP_PROP_FPS,          self.fps)
        self.get_logger().info("Cámara abierta correctamente")

    # ─────────────────────────────────────────
    # Detección de color
    # ─────────────────────────────────────────

    def _detectar(self, hsv, lower, upper, lower2=None, upper2=None):
        """
        Retorna (detected, x_norm, y_norm, area_norm, mask)
        x_norm: -1=izquierda, 0=centro, +1=derecha
        """
        h, w = hsv.shape[:2]

        mask = cv2.inRange(hsv, lower, upper)

        # Segundo rango opcional (para rojo que cruza 0/180)
        if lower2 is not None and upper2 is not None:
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask  = cv2.bitwise_or(mask, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid = [c for c in contours if cv2.contourArea(c) >= self.min_area]

        if not valid:
            return False, 0.0, 0.0, 0.0, mask

        target = max(valid, key=cv2.contourArea)
        area   = cv2.contourArea(target)

        M = cv2.moments(target)
        if M['m00'] == 0:
            return False, 0.0, 0.0, 0.0, mask

        cx        = int(M['m10'] / M['m00'])
        cy        = int(M['m01'] / M['m00'])
        x_norm    = (cx - w / 2) / (w / 2)
        y_norm    = cy / h
        area_norm = area / (h * w)

        return True, x_norm, y_norm, area_norm, mask

    # ─────────────────────────────────────────
    # Loop de captura
    # ─────────────────────────────────────────

    def capture_loop(self):
        while not self._stop:
            if not self.cap or not self.cap.isOpened():
                self._open_camera()
                import time; time.sleep(1.0)
                continue

            ret, frame = self.cap.read()
            if not ret:
                continue

            blurred = cv2.GaussianBlur(frame, (7, 7), 0)
            hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # ── Detectar verde ──
            v_det, vx, vy, va, v_mask = self._detectar(
                hsv, self.lower_verde, self.upper_verde)

            # ── Detectar café ──
            c_det, cx, cy, ca, c_mask = self._detectar(
                hsv, self.lower_cafe, self.upper_cafe)

            # ── Publicar el sargaso más grande como target principal ──
            # Prioridad: si hay verde Y café, publicar el de mayor área
            target_det = False
            tx, ty, ta = 0.0, 0.0, 0.0
            tipo = ''

            if v_det and c_det:
                if va >= ca:
                    tx, ty, ta, tipo = vx, vy, va, 'VERDE'
                else:
                    tx, ty, ta, tipo = cx, cy, ca, 'CAFE'
                target_det = True
            elif v_det:
                tx, ty, ta, tipo = vx, vy, va, 'VERDE'
                target_det = True
            elif c_det:
                tx, ty, ta, tipo = cx, cy, ca, 'CAFE'
                target_det = True

            if target_det:
                self.frames_sin_target = 0
                msg = Bool(); msg.data = True
                self.target_detected_pub.publish(msg)
                pos = Float32MultiArray()
                pos.data = [float(tx), float(ty), float(ta)]
                self.target_position_pub.publish(pos)
                self.get_logger().info(
                    f"{tipo} x={tx:+.2f} "
                    f"({'izq' if tx<-0.1 else 'der' if tx>0.1 else 'centro'}) "
                    f"area={ta:.3f}")
            else:
                self.frames_sin_target += 1
                if self.frames_sin_target >= self.FRAMES_PERDIDA:
                    msg = Bool(); msg.data = False
                    self.target_detected_pub.publish(msg)

            # ── Detectar boyas (rojo/naranja, doble rango) ──
            b_det, bx, by, ba, b_mask = self._detectar(
                hsv,
                self.lower_boya, self.upper_boya,
                self.lower_boya2, self.upper_boya2)

            if b_det:
                self.frames_sin_boya = 0
                msg = Bool(); msg.data = True
                self.boya_detected_pub.publish(msg)
                pos = Float32MultiArray()
                pos.data = [float(bx), float(by), float(ba)]
                self.boya_position_pub.publish(pos)
                self.get_logger().warn(
                    f"BOYA x={bx:+.2f} area={ba:.3f} — limite zona")
            else:
                self.frames_sin_boya += 1
                if self.frames_sin_boya >= self.FRAMES_PERDIDA:
                    msg = Bool(); msg.data = False
                    self.boya_detected_pub.publish(msg)

            # ── Imagen debug ──
            if self.pub_image:
                try:
                    annotated = frame.copy()
                    for c in cv2.findContours(
                            v_mask, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[0]:
                        if cv2.contourArea(c) >= self.min_area:
                            cv2.drawContours(annotated, [c], -1, (0, 255, 0), 2)
                    for c in cv2.findContours(
                            c_mask, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[0]:
                        if cv2.contourArea(c) >= self.min_area:
                            cv2.drawContours(annotated, [c], -1, (0, 165, 255), 2)
                    for c in cv2.findContours(
                            b_mask, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[0]:
                        if cv2.contourArea(c) >= self.min_area:
                            cv2.drawContours(annotated, [c], -1, (0, 0, 255), 2)
                    img_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
                    self.image_pub.publish(img_msg)
                except Exception as e:
                    self.get_logger().warn(f"Error imagen: {e}")

    def destroy_node(self):
        self._stop = True
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
