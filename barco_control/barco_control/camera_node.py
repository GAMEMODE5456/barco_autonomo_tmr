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
    Detecta dos colores de sargaso:
      Verde → recoger  → publica en 'target_detected' y 'target_position'
      Rojo  → evadir   → publica en 'danger_detected' y 'danger_position'
    """

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_index',      0)
        self.declare_parameter('frame_width',       640)
        self.declare_parameter('frame_height',      480)
        self.declare_parameter('fps',               30)
        self.declare_parameter('min_area',          500.0)
        self.declare_parameter('publish_image',     False)
        self.declare_parameter('hsv_lower_verde', [43, 50, 40])
        self.declare_parameter('hsv_upper_verde', [68, 255, 255])
        self.declare_parameter('hsv_lower_rojo',  [0, 100, 100])
        self.declare_parameter('hsv_upper_rojo',  [32, 255, 255])

        self.cam_idx    = self.get_parameter('camera_index').value
        self.frame_w    = self.get_parameter('frame_width').value
        self.frame_h    = self.get_parameter('frame_height').value
        self.fps        = self.get_parameter('fps').value
        self.min_area   = self.get_parameter('min_area').value
        self.pub_image  = self.get_parameter('publish_image').value

        self.lower_verde = np.array(self.get_parameter('hsv_lower_verde').value)
        self.upper_verde = np.array(self.get_parameter('hsv_upper_verde').value)
        self.lower_rojo  = np.array(self.get_parameter('hsv_lower_rojo').value)
        self.upper_rojo  = np.array(self.get_parameter('hsv_upper_rojo').value)

        # Publishers verde
        self.verde_detected_pub = self.create_publisher(Bool,             'target_detected', 10)
        self.verde_position_pub = self.create_publisher(Float32MultiArray,'target_position',  10)

        # Publishers rojo
        self.rojo_detected_pub  = self.create_publisher(Bool,             'danger_detected',  10)
        self.rojo_position_pub  = self.create_publisher(Float32MultiArray,'danger_position',   10)

        self.info_pub = self.create_publisher(String, 'target_info', 10)

        if self.pub_image:
            self.image_pub = self.create_publisher(Image, 'camera_image', 10)

        self.bridge = CvBridge()
        self.cap    = None
        self._stop  = False
        self._open_camera()

        self.frames_sin_verde = 0
        self.frames_sin_rojo  = 0
        self.FRAMES_PERDIDA   = 10

        self._thread = threading.Thread(target=self.capture_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(f"CameraNode listo — cam={self.cam_idx} {self.frame_w}x{self.frame_h}")
        self.get_logger().info(f"Verde lower={self.lower_verde} upper={self.upper_verde}")
        self.get_logger().info(f"Rojo  lower={self.lower_rojo}  upper={self.upper_rojo}")

    def _open_camera(self):
        self.cap = cv2.VideoCapture(self.cam_idx)
        if not self.cap.isOpened():
            self.get_logger().error(f"No se pudo abrir la cámara {self.cam_idx}")
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.frame_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
        self.cap.set(cv2.CAP_PROP_FPS,          self.fps)
        self.get_logger().info("Cámara abierta correctamente")

    def _detectar(self, hsv, lower, upper):
        h, w = hsv.shape[:2]
        mask = cv2.inRange(hsv, lower, upper)

        # Rojo cruza el 0/180 en HSV — agregar segundo rango
        if lower[0] <= 10:
            mask2 = cv2.inRange(
                hsv,
                np.array([170, lower[1], lower[2]]),
                np.array([180, upper[1], upper[2]]))
            mask = cv2.bitwise_or(mask, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) >= self.min_area]

        if not valid:
            return False, 0.0, 0.0, 0.0, mask

        target    = max(valid, key=cv2.contourArea)
        area      = cv2.contourArea(target)
        M         = cv2.moments(target)

        if M['m00'] == 0:
            return False, 0.0, 0.0, 0.0, mask

        cx        = int(M['m10'] / M['m00'])
        cy        = int(M['m01'] / M['m00'])
        x_norm    = (cx - w / 2) / (w / 2)
        y_norm    = cy / h
        area_norm = area / (h * w)

        return True, x_norm, y_norm, area_norm, mask

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

            # ── Verde ──
            v_det, vx, vy, va, v_mask = self._detectar(hsv, self.lower_verde, self.upper_verde)

            if v_det:
                self.frames_sin_verde = 0
                msg = Bool(); msg.data = True
                self.verde_detected_pub.publish(msg)
                pos = Float32MultiArray()
                pos.data = [float(vx), float(vy), float(va)]
                self.verde_position_pub.publish(pos)
                self.get_logger().info(
                    f"VERDE x={vx:+.2f} "
                    f"({'izq' if vx<-0.1 else 'der' if vx>0.1 else 'centro'}) "
                    f"area={va:.3f}")
            else:
                self.frames_sin_verde += 1
                if self.frames_sin_verde >= self.FRAMES_PERDIDA:
                    msg = Bool(); msg.data = False
                    self.verde_detected_pub.publish(msg)

            # ── Rojo ──
            r_det, rx, ry, ra, r_mask = self._detectar(hsv, self.lower_rojo, self.upper_rojo)

            if r_det:
                self.frames_sin_rojo = 0
                msg = Bool(); msg.data = True
                self.rojo_detected_pub.publish(msg)
                pos = Float32MultiArray()
                pos.data = [float(rx), float(ry), float(ra)]
                self.rojo_position_pub.publish(pos)
                self.get_logger().warn(f"ROJO (peligro) x={rx:+.2f} area={ra:.3f}")
            else:
                self.frames_sin_rojo += 1
                if self.frames_sin_rojo >= self.FRAMES_PERDIDA:
                    msg = Bool(); msg.data = False
                    self.rojo_detected_pub.publish(msg)

            # ── Imagen debug ──
            if self.pub_image:
                try:
                    annotated = frame.copy()
                    for c in cv2.findContours(v_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
                        if cv2.contourArea(c) >= self.min_area:
                            cv2.drawContours(annotated, [c], -1, (0, 255, 0), 2)
                    for c in cv2.findContours(r_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
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
