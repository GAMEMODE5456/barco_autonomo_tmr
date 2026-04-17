from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    """
    Lanza todos los nodos del barco autónomo TMR.

    Orden de arranque:
      1. esp_bridge_node  — primero, abre el serial
      2. Sensores (GPS, IMU, ultrasonico, camara) — 1s después
      3. Actuadores (motor, servo, conveyor) — 2s después
      4. navigation_node — 4s después (espera calibración IMU)
    """

    esp_bridge = Node(
        package='barco_control',
        executable='esp_bridge_node',
        name='esp_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baudrate':    115200,
        }]
    )

    gps = Node(
        package='barco_control',
        executable='gps_node',
        name='gps_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyAMA0',
            'baudrate':    9600,
        }]
    )

    imu = Node(
        package='barco_control',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'i2c_bus':          1,
            'publish_rate_hz':  20.0,
            'alpha':            0.98,
            'calibration_secs': 2.0,
        }]
    )

    ultrasonico = Node(
        package='barco_control',
        executable='ultra_sonic_node',
        name='ultra_sonic_node',
        output='screen',
        parameters=[{
            'trig_pin':        12,
            'echo_pin':        13,
            'measure_rate_hz': 5.0,
            'median_window':   5,
        }]
    )

    camara = Node(
        package='barco_control',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'camera_index':  0,
            'frame_width':   640,
            'frame_height':  480,
            'fps':           30,
            'min_area':      500.0,
            'publish_image': False,

            # ── Sargaso verde → recoger ──
            # Verde brillante amarillento
            'hsv_lower_verde': [35, 60, 60],
            'hsv_upper_verde': [75, 255, 255],

            # ── Sargaso café/morado → recoger ──
            # Tono rojizo-morado oscuro
            'hsv_lower_cafe': [120, 40, 30],
            'hsv_upper_cafe': [170, 180, 150],

            # ── Boyas rojo/naranja → límite de zona ──
            # Primer rango (naranja-rojo claro)
            'hsv_lower_boya':  [0, 150, 150],
            'hsv_upper_boya':  [15, 255, 255],
            # Segundo rango (rojo oscuro que cruza 180)
            'hsv_lower_boya2': [160, 150, 150],
            'hsv_upper_boya2': [180, 255, 255],
        }]
    )

    motor = Node(
        package='barco_control',
        executable='motor_node',
        name='motor_node',
        output='screen',
        parameters=[{
            'max_speed':     1.0,
            'watchdog_secs': 1.5,
        }]
    )

    servo = Node(
        package='barco_control',
        executable='servo_node',
        name='servo_node',
        output='screen',
        parameters=[{
            'min_angle':     -60.0,
            'max_angle':      60.0,
            'smooth_factor':  0.3,
        }]
    )

    conveyor = Node(
        package='barco_control',
        executable='conveyor_node',
        name='conveyor_node',
        output='screen',
        parameters=[{
            'run_duration': 0.0,   # 0 = indefinido (banda siempre activa)
            'cooldown':     0.0,
        }]
    )

    navigation = Node(
        package='barco_control',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{
            'search_turn_speed': 0.25,
            'approach_speed':    0.40,
            'collect_speed':     0.20,
            'max_rudder':        55.0,
            'rudder_kp':         55.0,
            'center_threshold':  0.15,
            'collect_area':      0.15,
            'obstacle_stop_cm':  30.0,
            'collect_time':       3.0,
            'search_timeout':    8.0,
            'boya_threshold':    0.6,
            'girar_time':        1.5,
        }]
    )

    return LaunchDescription([
        # Grupo 1: bridge serial
        esp_bridge,

        # Grupo 2: sensores — 1s después
        TimerAction(period=1.0, actions=[
            gps, imu, ultrasonico, camara
        ]),

        # Grupo 3: actuadores — 2s después
        TimerAction(period=2.0, actions=[
            motor, servo, conveyor
        ]),

        # Grupo 4: navegación — 4s después
        TimerAction(period=4.0, actions=[
            navigation
        ]),
    ])
