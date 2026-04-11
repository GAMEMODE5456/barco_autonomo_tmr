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
    Detecta objetos flotantes en el agua usando la webcam USB.

    Publica:
      'target_detected'  (Bool)            — hay objetivo en vista
      'target_position'  (Float32MultiArray) — [x_norm, y_norm, area_norm]
                          x_norm: -1.0 (izquierda) .. +1.0 (derecha)
                          y_norm:  0.0 (arriba)    .. +1.0 (abajo)
                          area_norm: 0.0 .. 1.0 (tamaño relativo al frame)
      'target_info'      (String)          — descripción del objeto detectado
      'camera_image'     (Image)           — frame con anotaciones (debug)

    Estrategia de detección:
      1. Filtro de color HSV — detecta colores específicos en agua
      2. Detección de contornos — encuentra formas flotantes
      3. Filtro de área mínima — descarta ruido pequeño
      4. El objeto más grande y centrado = objetivo principal

    Para cambiar el color a detectar ajusta HSV_LOWER y HSV_UPPER.
    Colores comunes para competencia:
      Rojo:    lower=(0,100,100)   upper=(10,255,255)  +  (170,100,100)-(180,255,255)
      Naranja: lower=(10,100,100)  upper=(25,255,255)
      Amarillo:lower=(25,100,100)  upper=(35,255,255)
      Verde:   lower=(40,50,50)    upper=(80,255,255)
      Azul:    lower=(100,100,50)  upper=(130,255,255)
    """

    def __init__(self):
        super().__init__('camera_node')

        # ── Parámetros ──
        self.declare_parameter('camera_index',   0)
        self.declare_parameter('frame_width',    640)
        self.declare_parameter('frame_height',   480)
        self.declare_parameter('fps',            30)
        self.declare_parameter('min_area',       500.0)   # px² mínimos para contar
        self.declare_parameter('publish_image',  True)    # False en producción

        # Color a detectar en HSV — AJUSTAR según las figuras de la competencia
        # Por defecto: naranja (buen contraste en agua)
        self.declare_parameter('hsv_lower', [10, 100, 100])
        self.declare_parameter('hsv_upper', [25, 255, 255])

        self.cam_idx      = self.get_parameter('camera_index').value
        self.frame_w      = self.get_parameter('frame_width').value
        self.frame_h      = self.get_parameter('frame_height').value
        self.fps          = self.get_parameter('fps').value
        self.min_area     = self.get_parameter('min_area').value
        self.pub_image    = self.get_parameter('publish_image').value
        self.hsv_lower    = np.array(self.get_parameter('hsv_lower').value)
        self.hsv_upper    = np.array(self.get_parameter('hsv_upper').value)

        # ── Publishers ──
        self.detected_pub = self.create_publisher(Bool,             'target_detected',  10)
        self.position_pub = self.create_publisher(Float32MultiArray,'target_position',  10)
        self.info_pub     = self.create_publisher(String,           'target_info',      10)
        if self.pub_image:
            self.image_pub = self.create_publisher(Image, 'camera_image', 10)

        self.bridge = CvBridge()

        # ── Cámara ──
        self.cap    = None
        self._stop  = False
        self._open_camera()

        # ── Estado ──
        self.target_detected  = False
        self.frames_sin_target = 0
        self.FRAMES_PERDIDA    = 10   # frames sin detección antes de publicar False

        # ── Hilo de captura ──
        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()

        self.get_logger().info(
            f"CameraNode listo — cam={self.cam_idx} "
            f"{self.frame_w}x{self.frame_h} @ {self.fps}fps")
        self.get_logger().info(
            f"Color HSV: lower={self.hsv_lower} upper={self.hsv_upper}")

    # ────────────────────────────────────────────
    # Inicializar cámara
    # ────────────────────────────────────────────

    def _open_camera(self):
        self.cap = cv2.VideoCapture(self.cam_idx)
        if not self.cap.isOpened():
            self.get_logger().error(
                f"No se pudo abrir la cámara {self.cam_idx}")
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.frame_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
        self.cap.set(cv2.CAP_PROP_FPS,          self.fps)
        self.get_logger().info("Cámara abierta correctamente")

    # ────────────────────────────────────────────
    # Detección de color en agua
    # ────────────────────────────────────────────

    def detect_target(self, frame: np.ndarray):
        """
        Retorna (detected, x_norm, y_norm, area_norm, annotated_frame).
        x_norm: -1=izquierda, 0=centro, +1=derecha
        y_norm:  0=arriba, 1=abajo
        area_norm: área relativa al frame completo
        """
        h, w = frame.shape[:2]
        frame_area = h * w

        # 1. Suavizar para reducir ruido del agua
        blurred = cv2.GaussianBlur(frame, (7, 7), 0)

        # 2. Convertir a HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 3. Máscara de color
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # Rojo tiene dos rangos en HSV (cruza el 0/180)
        # Si detectas rojo descomenta esto:
        # mask2 = cv2.inRange(hsv, np.array([170,100,100]), np.array([180,255,255]))
        # mask = cv2.bitwise_or(mask, mask2)

        # 4. Morfología para limpiar ruido del agua
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 5. Encontrar contornos
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        annotated = frame.copy()

        if not contours:
            return False, 0.0, 0.0, 0.0, annotated

        # 6. Filtrar por área mínima y tomar el más grande
        valid = [c for c in contours
                 if cv2.contourArea(c) >= self.min_area]

        if not valid:
            return False, 0.0, 0.0, 0.0, annotated

        # El contorno más grande es el objetivo principal
        target = max(valid, key=cv2.contourArea)
        area   = cv2.contourArea(target)

        # 7. Calcular centroide
        M = cv2.moments(target)
        if M['m00'] == 0:
            return False, 0.0, 0.0, 0.0, annotated

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # 8. Normalizar posición
        x_norm    = (cx - w / 2) / (w / 2)   # -1 .. +1
        y_norm    = cy / h                     #  0 .. 1
        area_norm = area / frame_area          #  0 .. 1

        # 9. Anotar frame para debug
        cv2.drawContours(annotated, [target], -1, (0, 255, 0), 2)
        cv2.circle(annotated, (cx, cy), 8, (0, 0, 255), -1)
        cv2.line(annotated, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)

        label = f"x={x_norm:.2f} area={area_norm:.3f}"
        cv2.putText(annotated, label, (cx + 10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Mostrar todos los candidatos válidos en gris
        for c in valid:
            if c is not target:
                cv2.drawContours(annotated, [c], -1, (128, 128, 128), 1)

        return True, x_norm, y_norm, area_norm, annotated

    # ────────────────────────────────────────────
    # Loop principal de captura
    # ────────────────────────────────────────────

    def capture_loop(self):
        while not self._stop:
            if not self.cap or not self.cap.isOpened():
                self.get_logger().warn("Cámara no disponible, reintentando...")
                self._open_camera()
                import time; import time as t; t.sleep(1.0)
                continue

            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Frame vacío de la cámara")
                continue

            detected, x_norm, y_norm, area_norm, annotated = \
                self.detect_target(frame)

            # ── Publicar detección ──
            if detected:
                self.frames_sin_target = 0

                det_msg = Bool()
                det_msg.data = True
                self.detected_pub.publish(det_msg)

                pos_msg = Float32MultiArray()
                pos_msg.data = [float(x_norm),
                                float(y_norm),
                                float(area_norm)]
                self.position_pub.publish(pos_msg)

                info_msg = String()
                info_msg.data = (
                    f"x={x_norm:.3f} y={y_norm:.3f} "
                    f"area={area_norm:.4f}")
                self.info_pub.publish(info_msg)

                self.get_logger().info(
                    f"Objetivo: x={x_norm:+.2f} "
                    f"({'izq' if x_norm < -0.1 else 'der' if x_norm > 0.1 else 'centro'}) "
                    f"area={area_norm:.3f}")

            else:
                self.frames_sin_target += 1
                if self.frames_sin_target >= self.FRAMES_PERDIDA:
                    det_msg = Bool()
                    det_msg.data = False
                    self.detected_pub.publish(det_msg)
                    if self.frames_sin_target == self.FRAMES_PERDIDA:
                        self.get_logger().info("Objetivo perdido")

            # ── Publicar imagen anotada (modo debug) ──
            if self.pub_image:
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
                    self.image_pub.publish(img_msg)
                except Exception as e:
                    self.get_logger().warn(f"Error publicando imagen: {e}")

    # ────────────────────────────────────────────
    # Destructor
    # ────────────────────────────────────────────

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
    node.destroy_node()
    rclpy.shutdown()
