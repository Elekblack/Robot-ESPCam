/*
  ESP32-C3 Super Mini: controlador de motores y brazo (esclava I2C)
  Recibe órdenes de la ESP32-CAM (maestra) y maneja:
    - 2 motores DC con driver MX1508
    - 2 servos SG90 (brazo pan/tilt para la cámara)
  Compatible con ESP32 Arduino core 3.x.

  PROTOCOLO: paquetes de 3 bytes [tipo, dato1, dato2]
    'M' (mover): dato1 = comando (F B L R Q E Z C S), dato2 = velocidad 0-255
    'A' (brazo): dato1 = X 0-100 (pan), dato2 = Y 0-100 (tilt)

  SEGURIDAD: si no llega ningún mensaje 'M' en TIMEOUT_MS, los motores se detienen
  (la página web repite el comando mientras se mantiene presionado el botón).

  CONEXIONES (ESP32-C3 Super Mini)
    GPIO 5  (SDA) <- GPIO 13 de la ESP32-CAM
    GPIO 6  (SCL) <- GPIO 14 de la ESP32-CAM
    GND           <- GND de la ESP32-CAM (masa común)
    GPIO 4  -> MX1508 IN1 (motor izq A)
    GPIO 3  -> MX1508 IN2 (motor izq B)
    GPIO 1  -> MX1508 IN3 (motor der A)
    GPIO 0 -> MX1508 IN4 (motor der B)
    GPIO 10  -> servo pan  (horizontal), señal naranja
    GPIO 20  -> servo tilt (vertical),   señal naranja
    Servos VCC -> 5 V del regulador, GND -> GND común (capacitor 470-1000 uF cerca)
    MX1508 VCC -> batería (7.4 V), GND -> GND común
    Pull-ups I2C: 4.7 kOhm de SDA a 3.3 V y de SCL a 3.3 V (un solo par en todo el bus)

  ARDUINO IDE
    Placa: "ESP32C3 Dev Module" (o "Nologo ESP32C3 Super Mini")
    Este sketch no usa Serial.
    El LED azul integrado (GPIO 8) cambia de estado con cada mensaje recibido:
    sirve para comprobar que el enlace I2C funciona.
*/

#include <Wire.h>

// ================== CONFIGURACIÓN ==================
// I2C (debe coincidir con C3_ADDR de la ESP32-CAM)
const uint8_t  I2C_ADDR = 0x30;
const int      PIN_SDA  = 5;
const int      PIN_SCL  = 6;
const uint32_t I2C_FREQ = 100000;

// Motores (MX1508)
const int M_IZQ_A = 3;
const int M_IZQ_B = 4;
const int M_DER_A = 0;
const int M_DER_B = 1;
const bool INVERTIR_IZQ = false;        // pon true si un motor gira al revés
const bool INVERTIR_DER = false;
const uint32_t PWM_FREQ = 2000;         // Hz
const uint8_t  PWM_BITS = 8;            // 0-255

// Brazo de la cámara: 2 servos SG90
const int PIN_PAN  = 10;
const int PIN_TILT = 20;
const bool INVERTIR_PAN  = false;       // pon true si la cámara se mueve al revés que el dedo
const bool INVERTIR_TILT = false;
const int PAN_MIN  = 0,  PAN_MAX  = 180;
const int TILT_MIN = 40, TILT_MAX = 140; // límites para que el brazo no choque con el robot
const int PULSO_MIN_US = 500;           // pulso para 0°   (SG90: 500 us)
const int PULSO_MAX_US = 2400;          // pulso para 180° (SG90: 2400 us)
const int PASO_SERVO = 3;               // grados cada 20 ms (menor = más lento y suave)
const uint8_t SERVO_BITS = 14;          // la C3 admite como máximo 14 bits de resolución

// LEDC: la C3 tiene 6 canales. 0-3 motores (2 kHz), 4-5 servos (50 Hz)
const int CH_IZQ_A = 0, CH_IZQ_B = 1, CH_DER_A = 2, CH_DER_B = 3, CH_PAN = 4, CH_TILT = 5;

// Si no llega ningún comando de movimiento en este tiempo, los motores se detienen
const unsigned long TIMEOUT_MS = 400;

// LED azul integrado
const int PIN_LED = 8;

// ================== ESTADO ==================
// Los datos llegan en alRecibir() y se aplican en loop() (el callback se mantiene corto)
volatile uint16_t rxMotor = 0;          // (comando << 8) | velocidad
volatile uint16_t rxBrazo = 0;          // (x << 8) | y
volatile bool hayMotor = false;
volatile bool hayBrazo = false;

unsigned long ultimoMotor = 0;
bool motoresActivos = false;

int panObjetivo  = (PAN_MIN + PAN_MAX) / 2;
int tiltObjetivo = (TILT_MIN + TILT_MAX) / 2;
int panActual  = (PAN_MIN + PAN_MAX) / 2;
int tiltActual = (TILT_MIN + TILT_MAX) / 2;

// ================== MOTORES ==================
void setMotor(int pinA, int pinB, int v) {
  v = constrain(v, -255, 255);
  if (v > 0)      { ledcWrite(pinA, v);  ledcWrite(pinB, 0);  }
  else if (v < 0) { ledcWrite(pinA, 0);  ledcWrite(pinB, -v); }
  else            { ledcWrite(pinA, 0);  ledcWrite(pinB, 0);  }
}

void mover(int izq, int der) {
  if (INVERTIR_IZQ) izq = -izq;
  if (INVERTIR_DER) der = -der;
  setMotor(M_IZQ_A, M_IZQ_B, izq);
  setMotor(M_DER_A, M_DER_B, der);
  motoresActivos = (izq != 0 || der != 0);
}

void ejecutarComando(char c, int s) {
  s = constrain(s, 0, 255);
  int lento = s / 3;                    // rueda "interior" en las curvas
  switch (c) {
    case 'F': mover(s, s);        break;  // adelante
    case 'B': mover(-s, -s);      break;  // atrás
    case 'L': mover(-s, s);       break;  // gira a la izquierda sobre su eje
    case 'R': mover(s, -s);       break;  // gira a la derecha sobre su eje
    case 'Q': mover(lento, s);    break;  // adelante-izquierda
    case 'E': mover(s, lento);    break;  // adelante-derecha
    case 'Z': mover(-lento, -s);  break;  // atrás-izquierda
    case 'C': mover(-s, -lento);  break;  // atrás-derecha
    default:  mover(0, 0);        break;  // 'S' o desconocido = stop
  }
}

// ================== BRAZO (SERVOS SG90) ==================
// Servo por LEDC: 50 Hz, 14 bits -> el periodo de 20000 us equivale a 16384 pasos
void escribirServo(int pin, int angulo, bool invertir) {
  angulo = constrain(angulo, 0, 180);
  if (invertir) angulo = 180 - angulo;
  uint32_t us = map(angulo, 0, 180, PULSO_MIN_US, PULSO_MAX_US);
  ledcWrite(pin, (us * 16384UL) / 20000UL);
}

void moverBrazo(int pan, int tilt) {
  panObjetivo  = constrain(pan,  PAN_MIN,  PAN_MAX);
  tiltObjetivo = constrain(tilt, TILT_MIN, TILT_MAX);
}

// Se llama cada 20 ms: acerca los servos al objetivo de a PASO_SERVO grados
void actualizarBrazo() {
  if (panObjetivo != panActual) {
    panActual += constrain(panObjetivo - panActual, -PASO_SERVO, PASO_SERVO);
    escribirServo(PIN_PAN, panActual, INVERTIR_PAN);
  }
  if (tiltObjetivo != tiltActual) {
    tiltActual += constrain(tiltObjetivo - tiltActual, -PASO_SERVO, PASO_SERVO);
    escribirServo(PIN_TILT, tiltActual, INVERTIR_TILT);
  }
}

// ================== I2C ==================
// Callback del esclavo: solo guarda el paquete; loop() lo procesa
void alRecibir(int n) {
  uint8_t buf[3];
  int i = 0;
  while (Wire.available()) {
    int b = Wire.read();
    if (i < 3) buf[i++] = (uint8_t)b;
  }
  if (i != 3) return;                   // paquete incompleto: se ignora
  uint16_t dato = ((uint16_t)buf[1] << 8) | buf[2];
  if (buf[0] == 'M')      { rxMotor = dato; hayMotor = true; }
  else if (buf[0] == 'A') { rxBrazo = dato; hayBrazo = true; }
}

void latido() {                         // el LED cambia con cada mensaje recibido
  static bool estado = false;
  estado = !estado;
  digitalWrite(PIN_LED, estado);
}

// ================== SETUP / LOOP ==================
void setup() {
  pinMode(PIN_LED, OUTPUT);

  // Motores: core 3.x -> ledcAttachChannel(pin, freq, bits, canal)
  ledcAttachChannel(M_IZQ_A, PWM_FREQ, PWM_BITS, CH_IZQ_A);
  ledcAttachChannel(M_IZQ_B, PWM_FREQ, PWM_BITS, CH_IZQ_B);
  ledcAttachChannel(M_DER_A, PWM_FREQ, PWM_BITS, CH_DER_A);
  ledcAttachChannel(M_DER_B, PWM_FREQ, PWM_BITS, CH_DER_B);
  mover(0, 0);

  // Servos: 50 Hz, 14 bits; arrancan centrados
  ledcAttachChannel(PIN_PAN,  50, SERVO_BITS, CH_PAN);
  ledcAttachChannel(PIN_TILT, 50, SERVO_BITS, CH_TILT);
  escribirServo(PIN_PAN,  panActual,  INVERTIR_PAN);
  escribirServo(PIN_TILT, tiltActual, INVERTIR_TILT);

  // I2C esclavo (en core 3.x el callback se registra antes de begin)
  Wire.onReceive(alRecibir);
  if (!Wire.begin(I2C_ADDR, PIN_SDA, PIN_SCL, I2C_FREQ)) {
    while (true) {                      // error: LED parpadeando rápido
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(100);
    }
  }
}

void loop() {
  if (hayMotor) {
    hayMotor = false;
    uint16_t d = rxMotor;
    ejecutarComando((char)(d >> 8), d & 0xFF);
    ultimoMotor = millis();
    latido();
  }

  if (hayBrazo) {
    hayBrazo = false;
    uint16_t d = rxBrazo;
    int x = constrain((int)(d >> 8), 0, 100);
    int y = constrain((int)(d & 0xFF), 0, 100);
    moverBrazo(map(x, 0, 100, PAN_MIN, PAN_MAX), map(y, 0, 100, TILT_MIN, TILT_MAX));
    latido();
  }

  // Failsafe: si se pierde la conexión o se suelta el botón, el robot se detiene
  if (motoresActivos && (millis() - ultimoMotor > TIMEOUT_MS)) {
    mover(0, 0);
  }

  static unsigned long tServo = 0;
  if (millis() - tServo >= 20) {        // movimiento suave del brazo
    tServo = millis();
    actualizarBrazo();
  }

  delay(2);
}
