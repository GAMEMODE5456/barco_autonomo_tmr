# Barco Autónomo TMR México

Barco autónomo para competencia TMR México.
**Stack:** ROS2 Humble + Raspberry Pi 5 + ESP32-S3

---

## Estructura del repositorio

```
barco_autonomo_tmr/
│
├── barco_control/                  # Paquete ROS2 principal
│   ├── barco_control/              # Código fuente de los nodos
│   │   ├── __init__.py
│   │   ├── camera_node.py          # Detección de objetos con webcam (OpenCV HSV)
│   │   ├── navigation_node.py      # Lógica autónoma (máquina de estados)
│   │   ├── esp_bridge_node.py      # Puente serial Raspberry ↔ ESP32
│   │   ├── motor_node.py           # Control de motores (diferencial)
│   │   ├── servo_node.py           # Control del servo timón
│   │   ├── conveyor_node.py        # Banda transportadora
│   │   ├── gps_node.py             # GPS NEO-M8N (NMEA corregido)
│   │   ├── imu_node.py             # IMU MPU6050 (filtro complementario)
│   │   └── ultra_sonic_node.py     # HC-SR04 + fusión con ESP32
│   │
│   ├── launch/
│   │   └── barco_launch.py         # Lanza todos los nodos
│   │
│   ├── resource/
│   │   └── barco_control
│   ├── test/
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
├── esp32/
│   └── esp32_barco.ino             # Firmware ESP32-S3
│
└── README.md
```

---

## Hardware

| Componente | Modelo | Conexión |
|---|---|---|
| Computadora | Raspberry Pi 5 | — |
| Microcontrolador | ESP32-S3 DevKitC-1 N16R8 | USB-C → Raspi |
| Driver PWM | PCA9685 | I2C GPIO8/9 del ESP32 |
| Motores propulsión x2 | Brushless/DC | ESC 50A bidi → PCA CH0/CH1 |
| Timón | Servo | PCA CH2, rail 6V |
| Banda transportadora | Motorreductor amarillo | L298N → ESP32 GPIO10/11 |
| GPS | NEO-M8N GPS+GLONASS | UART /dev/ttyAMA0 |
| IMU | MPU6050 | I2C Raspi bus 1 |
| Ultrasonido | HC-SR04 | GPIO12(TRIG) GPIO13(ECHO) |
| Cámara | Webcam USB | USB-A Raspi |
| Batería | LiFePO4 12V 20800mAh | bus principal |
| Reg. lógica | 12V→5V 6A | Raspi 5V + PCA VCC |
| Reg. servos | 12V→6V 10A | PCA rail V+ + servo |

---

## Instalación en Raspberry Pi 5

### 1. Requisitos del sistema

```bash
# ROS2 Humble
sudo apt update && sudo apt install -y ros-humble-desktop

# Dependencias Python
pip install smbus2 pyserial opencv-python RPi.GPIO --break-system-packages

# Dependencias ROS2
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-sensor-msgs \
  python3-rpi.gpio
```

### 2. Clonar y compilar

```bash
cd ~
git clone https://github.com/GAMEMODE5456/barco_autonomo_tmr.git
cd barco_autonomo_tmr

# Compilar
source /opt/ros/humble/setup.bash
colcon build --packages-select barco_control
source install/setup.bash
```

### 3. Copiar firmware al ESP32

Abrir `esp32/esp32_barco.ino` en Arduino IDE y subir al ESP32-S3.
Librería requerida: `Adafruit PWM Servo Driver Library`

---

## Uso

### Arrancar todo el sistema

```bash
source /opt/ros/humble/setup.bash
source ~/barco_autonomo_tmr/install/setup.bash
ros2 launch barco_control barco_launch.py
```

### Arrancar nodos individuales (debug)

```bash
# Solo cámara para calibrar color HSV
ros2 run barco_control camera_node \
  --ros-args -p publish_image:=true \
             -p hsv_lower:=[10,100,100] \
             -p hsv_upper:=[25,255,255]

# Ver imagen de la cámara con detecciones
ros2 run rqt_image_view rqt_image_view /camera_image

# Ver estado de navegación
ros2 topic echo /nav_state

# Ver distancia de obstáculos
ros2 topic echo /obstacle_distance

# Mover timón manualmente para prueba
ros2 topic pub /rudder_angle std_msgs/Float32 "data: 20.0"

# Mover motores manualmente
ros2 topic pub /motor_speeds std_msgs/Float32MultiArray \
  "data: [0.3, 0.3]"

# Activar banda manualmente
ros2 topic pub /conveyor_power std_msgs/Bool "data: true"
```

---

## Calibración de color HSV (importante)

El `camera_node` detecta objetos por color. Antes de la competencia
hay que calibrar el color de las figuras:

```bash
# Ver el frame con las detecciones en tiempo real
ros2 run barco_control camera_node \
  --ros-args -p publish_image:=true

# En otra terminal ver la imagen
ros2 run rqt_image_view rqt_image_view /camera_image
```

Referencia de rangos HSV comunes:

| Color | HSV lower | HSV upper |
|---|---|---|
| Naranja | [10, 100, 100] | [25, 255, 255] |
| Rojo | [0, 100, 100] | [10, 255, 255] |
| Amarillo | [25, 100, 100] | [35, 255, 255] |
| Verde | [40, 50, 50] | [80, 255, 255] |
| Azul | [100, 100, 50] | [130, 255, 255] |

---

## Protocolo serial Raspberry ↔ ESP32

| Dirección | Formato | Descripción |
|---|---|---|
| Raspi → ESP32 | `MOT:L:0.50,R:0.50\n` | Velocidad motores [-1..1] |
| Raspi → ESP32 | `SRV:15.0\n` | Ángulo timón [-45..45]° |
| Raspi → ESP32 | `CONV:ON\n` | Banda transportadora ON |
| Raspi → ESP32 | `CONV:OFF\n` | Banda transportadora OFF |
| ESP32 → Raspi | `DIST:32.5\n` | Distancia ultrasonido cm |
| ESP32 → Raspi | `CONV:ON\n` | Confirmación estado banda |
| ESP32 → Raspi | `HB:OK\n` | Heartbeat cada 5s |

---

## Topics ROS2

| Topic | Tipo | Publicado por | Suscrito por |
|---|---|---|---|
| `/target_detected` | Bool | camera_node | navigation_node |
| `/target_position` | Float32MultiArray | camera_node | navigation_node |
| `/gps_data` | NavSatFix | gps_node | navigation_node |
| `/imu_data` | Imu | imu_node | navigation_node |
| `/imu_yaw` | Float32 | imu_node | — |
| `/obstacle_distance` | Float32 | ultra_sonic_node (fusión) | navigation_node |
| `/obstacle_distance_sonar` | Float32 | ultra_sonic_node | — |
| `/obstacle_distance_esp` | Float32 | esp_bridge_node | ultra_sonic_node |
| `/motor_speeds` | Float32MultiArray | motor_node | esp_bridge_node |
| `/rudder_angle` | Float32 | navigation_node | servo_node, esp_bridge_node |
| `/conveyor_power` | Bool | navigation_node | conveyor_node, esp_bridge_node |
| `/conveyor_status` | Bool | esp_bridge_node | — |
| `/nav_state` | String | navigation_node | — |
| `/camera_image` | Image | camera_node (debug) | rqt_image_view |

---

## Parámetros clave para la competencia

Ajustar en `barco_launch.py` antes de competir:

```python
# camera_node — color de las figuras
'hsv_lower': [10, 100, 100],   # cambiar según figuras reales
'hsv_upper': [25, 255, 255],

# navigation_node — comportamiento
'search_turn_speed': 0.25,     # qué tan rápido gira buscando
'approach_speed':    0.45,     # velocidad de acercamiento
'rudder_kp':         40.0,     # agresividad del timón
'collect_area':      0.15,     # qué tan cerca para recoger
'obstacle_stop_cm':  40.0,     # distancia de parada de emergencia
```
