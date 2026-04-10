/*
  Barco Autónomo TMR — ESP32-S3
  ==============================
  Recibe comandos seriales desde Raspberry Pi (ROS2) y controla:
    - 2 motores de propulsión via PCA9685 CH0 y CH1 (ESC 50A)
    - Servo timón via PCA9685 CH2 (antes era CH1 — ajustado)
    - Banda transportadora via L298N conectado a pines GPIO del ESP32
    - Sensor ultrasónico HC-SR04

  Protocolo serial (Raspberry → ESP32):
    MOT:L:0.50,R:0.50\n   velocidad motores [-1.00 .. 1.00]
    SRV:15.0\n             ángulo servo [-45.0 .. 45.0]
    CONV:ON\n              encender banda
    CONV:OFF\n             apagar banda

  Protocolo serial (ESP32 → Raspberry):
    DIST:32.5\n            distancia ultrasónico en cm
    CONV:ON\n / CONV:OFF\n confirmación estado banda
    HB:OK\n                heartbeat cada 5s
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ── Canales PCA9685 ──
#define CH_MOTOR_IZQ  0   // ESC motor izquierdo
#define CH_MOTOR_DER  1   // ESC motor derecho
#define CH_TIMON      2   // Servo timón

// ── Valores PWM para ESC (ajustar según calibración) ──
#define ESC_NEUTRO    307
#define ESC_MIN       260   // reversa máxima
#define ESC_MAX       360   // adelante máximo

// ── Servo timón ──
#define SERVO_CENTRO  307
#define SERVO_MIN     250   // izquierda máxima
#define SERVO_MAX     370   // derecha máxima

// ── L298N para banda transportadora ──
// Conectar: IN1 y IN2 del L298N a estos pines del ESP32
// ENA del L298N a 5V (o a un GPIO para control de velocidad)
#define CONV_IN1  10    // GPIO10 → L298N IN1
#define CONV_IN2  11    // GPIO11 → L298N IN2
// Motor DC del L298N → motorreductor amarillo

// ── Ultrasónico HC-SR04 ──
#define TRIG_PIN  12
#define ECHO_PIN  13

// ── Estado ──
bool  conveyorOn     = false;
float motorIzq       = 0.0;
float motorDer       = 0.0;
float timonAngulo    = 0.0;

unsigned long lastCmd        = 0;
unsigned long lastHeartbeat  = 0;
const unsigned long WATCHDOG_MS   = 1500;  // ms sin comando → stop
const unsigned long HEARTBEAT_MS  = 5000;

// ────────────────────────────────────────────
// Conversión velocidad [-1,1] → pulso PCA9685
// ────────────────────────────────────────────
int speedToPWM(float speed) {
  // speed: -1.0 = reversa max, 0.0 = neutro, 1.0 = adelante max
  speed = constrain(speed, -1.0, 1.0);
  if (speed >= 0)
    return (int)(ESC_NEUTRO + speed * (ESC_MAX - ESC_NEUTRO));
  else
    return (int)(ESC_NEUTRO + speed * (ESC_NEUTRO - ESC_MIN));
}

// ────────────────────────────────────────────
// Conversión ángulo [-45,45] → pulso PCA9685
// ────────────────────────────────────────────
int angleToPWM(float angle) {
  angle = constrain(angle, -45.0, 45.0);
  return (int)(SERVO_CENTRO + (angle / 45.0) * (SERVO_MAX - SERVO_CENTRO));
}

// ────────────────────────────────────────────
// Control motores
// ────────────────────────────────────────────
void setMotores(float izq, float der) {
  motorIzq = izq;
  motorDer = der;
  pwm.setPWM(CH_MOTOR_IZQ, 0, speedToPWM(izq));
  pwm.setPWM(CH_MOTOR_DER, 0, speedToPWM(der));
}

void stopMotores() {
  setMotores(0.0, 0.0);
}

// ────────────────────────────────────────────
// Control timón
// ────────────────────────────────────────────
void setTimon(float angle) {
  timonAngulo = angle;
  pwm.setPWM(CH_TIMON, 0, angleToPWM(angle));
}

// ────────────────────────────────────────────
// Control banda transportadora (L298N)
// ────────────────────────────────────────────
void setBanda(bool on) {
  conveyorOn = on;
  if (on) {
    // Motor hacia adelante: IN1=HIGH, IN2=LOW
    digitalWrite(CONV_IN1, HIGH);
    digitalWrite(CONV_IN2, LOW);
    Serial.println("CONV:ON");
  } else {
    // Motor frenado: IN1=LOW, IN2=LOW
    digitalWrite(CONV_IN1, LOW);
    digitalWrite(CONV_IN2, LOW);
    Serial.println("CONV:OFF");
  }
}

// ────────────────────────────────────────────
// Sensor ultrasónico
// ────────────────────────────────────────────
float medirDistancia() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duracion = pulseIn(ECHO_PIN, HIGH, 30000);  // timeout 30ms
  if (duracion == 0) return 999.0;                 // sin eco = lejos
  return (duracion * 0.0343) / 2.0;               // cm
}

// ────────────────────────────────────────────
// Parser de comandos seriales
// ────────────────────────────────────────────
void procesarLinea(String line) {
  line.trim();
  if (line.length() == 0) return;

  // MOT:L:0.50,R:-0.30
  if (line.startsWith("MOT:")) {
    // Extraer L y R
    int li = line.indexOf("L:");
    int ri = line.indexOf(",R:");
    if (li < 0 || ri < 0) return;

    float l = line.substring(li + 2, ri).toFloat();
    float r = line.substring(ri + 3).toFloat();
    setMotores(l, r);
    lastCmd = millis();
    return;
  }

  // SRV:15.0
  if (line.startsWith("SRV:")) {
    float angle = line.substring(4).toFloat();
    setTimon(angle);
    lastCmd = millis();
    return;
  }

  // CONV:ON / CONV:OFF
  if (line.startsWith("CONV:")) {
    String estado = line.substring(5);
    estado.toUpperCase();
    setBanda(estado == "ON");
    lastCmd = millis();
    return;
  }
}

// ────────────────────────────────────────────
// Setup
// ────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // I2C para PCA9685
  Wire.begin(8, 9);   // SDA=GPIO8, SCL=GPIO9
  pwm.begin();
  pwm.setPWMFreq(50);

  // Posición inicial
  stopMotores();
  setTimon(0.0);

  // L298N pines
  pinMode(CONV_IN1, OUTPUT);
  pinMode(CONV_IN2, OUTPUT);
  setBanda(false);

  // Ultrasónico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  lastCmd = millis();
  Serial.println("HB:BOOT");
  Serial.println("ESP32 listo");
}

// ────────────────────────────────────────────
// Loop
// ────────────────────────────────────────────
void loop() {
  // Leer comandos seriales
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    procesarLinea(line);
  }

  // Watchdog: sin comandos → stop motores (banda se deja como está)
  if (millis() - lastCmd > WATCHDOG_MS) {
    if (abs(motorIzq) > 0.01 || abs(motorDer) > 0.01) {
      stopMotores();
      Serial.println("WDG:STOP");
    }
  }

  // Ultrasónico cada 200ms
  static unsigned long lastSonar = 0;
  if (millis() - lastSonar > 200) {
    float dist = medirDistancia();
    Serial.print("DIST:");
    Serial.println(dist, 1);
    lastSonar = millis();
  }

  // Heartbeat cada 5s
  if (millis() - lastHeartbeat > HEARTBEAT_MS) {
    Serial.println("HB:OK");
    lastHeartbeat = millis();
  }
}
