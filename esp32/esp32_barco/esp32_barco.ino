/*
  Barco Autónomo TMR — ESP32-S3
  ==============================
  Recibe comandos seriales desde Raspberry Pi (ROS2) y controla:
    - 1 motor de propulsión via PCA9685 CH0 (ESC 50A)
    - Servo timón via PCA9685 CH1  ← rango ampliado a ±60°
    - Banda transportadora via L298N conectado a pines GPIO del ESP32
    - Sensor ultrasónico HC-SR04

  Protocolo serial (Raspberry → ESP32):
    MOT:0.50\n             velocidad motor [-1.00 .. 1.00]
    SRV:30.0\n             ángulo servo  [-60.0 .. 60.0]
    CONV:ON\n              encender banda
    CONV:OFF\n             apagar banda

  Protocolo serial (ESP32 → Raspberry):
    DIST:32.5\n            distancia ultrasónico en cm
    CONV:ON\n / CONV:OFF\n confirmación estado banda
    HB:OK\n                heartbeat cada 5s
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ── Canales PCA9685 ──
#define CH_MOTOR   0   // ESC motor único
#define CH_TIMON   1   // Servo timón

// ── Valores PWM para ESC (ajustar según calibración) ──
#define ESC_NEUTRO  310
#define ESC_MIN     260   // reversa máxima
#define ESC_MAX     360   // adelante máximo

// ── Servo timón — rango ampliado a ±60° ──
// Ajusta SERVO_MIN y SERVO_MAX según el recorrido físico real del servo.
// Partiendo del rango anterior ±45° → [250, 370], se extrapoló a ±60°.
#define SERVO_CENTRO  307
#define SERVO_MIN     232   // izquierda máxima  (-60°)
#define SERVO_MAX     382   // derecha máxima    (+60°)

// ── L298N para banda transportadora ──
#define CONV_IN1  10    // GPIO10 → L298N IN1
#define CONV_IN2  11    // GPIO11 → L298N IN2

// ── Ultrasónico HC-SR04 ──
#define TRIG_PIN  12
#define ECHO_PIN  13

// ── Estado ──
bool  conveyorOn  = false;
float motorVel    = 0.0;
float timonAngulo = 0.0;

unsigned long lastCmd       = 0;
unsigned long lastHeartbeat = 0;
const unsigned long WATCHDOG_MS  = 1500;
const unsigned long HEARTBEAT_MS = 5000;

// ────────────────────────────────────────────
// Conversión velocidad [-1,1] → pulso PCA9685
// ────────────────────────────────────────────
int speedToPWM(float speed) {
  speed = constrain(speed, -1.0, 1.0);
  if (speed >= 0)
    return (int)(ESC_NEUTRO + speed * (ESC_MAX - ESC_NEUTRO));
  else
    return (int)(ESC_NEUTRO + speed * (ESC_NEUTRO - ESC_MIN));
}

// ────────────────────────────────────────────
// Conversión ángulo [-60,60] → pulso PCA9685
// ────────────────────────────────────────────
int angleToPWM(float angle) {
  angle = constrain(angle, -60.0, 60.0);
  return (int)(SERVO_CENTRO + (angle / 60.0) * (SERVO_MAX - SERVO_CENTRO));
}

// ────────────────────────────────────────────
// Control motor
// ────────────────────────────────────────────
void setMotor(float vel) {
  motorVel = vel;
  pwm.setPWM(CH_MOTOR, 0, speedToPWM(vel));
}

void stopMotor() {
  setMotor(0.0);
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
    digitalWrite(CONV_IN1, HIGH);
    digitalWrite(CONV_IN2, LOW);
    Serial.println("CONV:ON");
  } else {
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

  long duracion = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duracion == 0) return 999.0;
  return (duracion * 0.0343) / 2.0;  // cm
}

// ────────────────────────────────────────────
// Parser de comandos seriales
// ────────────────────────────────────────────
void procesarLinea(String line) {
  line.trim();
  if (line.length() == 0) return;

  // MOT:0.50  (un solo valor)
  if (line.startsWith("MOT:")) {
    float vel = line.substring(4).toFloat();
    setMotor(vel);
    lastCmd = millis();
    return;
  }

  // SRV:30.0  (rango -60 .. 60)
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
  Wire.begin(8, 9);
  pwm.begin();
  pwm.setPWMFreq(50);

  delay(500);

  // Secuencia de armado ESC ZMR
  pwm.setPWM(CH_MOTOR, 0, ESC_MAX);   // maximo
  delay(2000);
  pwm.setPWM(CH_MOTOR, 0, ESC_NEUTRO); // neutro
  setTimon(0.0);
  delay(2000);

  pinMode(CONV_IN1, OUTPUT);
  pinMode(CONV_IN2, OUTPUT);
  setBanda(false);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  lastCmd = millis();
  Serial.println("HB:BOOT");
  Serial.println("ESP32 listo — 1 motor, timon +/-60deg");
}

// ────────────────────────────────────────────
// Loop
// ────────────────────────────────────────────
void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    procesarLinea(line);
  }
  // Debug: mostrar todo lo que llega
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    Serial.print("Recibido: [");
    Serial.print(line);
    Serial.println("]");
    procesarLinea(line);
  }
  // Watchdog: sin comandos → stop motor
  //if (millis() - lastCmd > WATCHDOG_MS) {
    //if (abs(motorVel) > 0.01) {
      //stopMotor();
      //Serial.println("WDG:STOP");
    //}
  //}

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
