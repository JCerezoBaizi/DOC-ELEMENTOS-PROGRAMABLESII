# Proyecto Final ESP32 – Plataforma Stewart

## 1) Resumen

- **Nombre del proyecto:** _Control de plataforma Stewart con ESP32 y visión por computadora_  
- **Equipo / Autor(es):** _Jesus Cerezo - Emmanuel Angeles-Ximena Verdi_  
- **Curso / Asignatura:** _Elementos programables II_  
- **Fecha:** _08/12/25_  
- **Descripción breve:** _Este proyecto controla una pelota sobre una plataforma tipo Stewart utilizando tres servomotores manejados por un ESP32. La inclinación de la base se controla con el movimiento de la mano, detectado mediante cámara y MediaPipe, y los ángulos se envían al ESP32 por Bluetooth para posicionar suavemente la plataforma._

## 2) Objetivos

- **General:** _Desarrollar un sistema de control de plataforma tipo Stewart utilizando un ESP32 y visión por computadora, que permita mover una pelota mediante la inclinación de la mano del usuario._

- **Específicos:**
  - _Configurar un ESP32 para controlar tres servomotores mediante PWM y comunicación Bluetooth._
  - _Implementar en Python la detección de mano con MediaPipe para obtener inclinaciones tipo pitch y roll._
  - _Mapear los movimientos de la mano a ángulos de servos y enviarlos al ESP32 en tiempo real._
  - _Diseñar e imprimir en 3D la estructura mecánica (base, brazos y uniones) para soportar la plataforma._

## 3) Alcance y Exclusiones

- **Incluye:**
  - Control en tiempo real de tres servomotores con rampas suaves de movimiento.
  - Comunicación Bluetooth entre computadora (Python) y ESP32.
  - Detección de mano con cámara web usando MediaPipe.
  - Diseño e impresión 3D de la base y brazos de la plataforma.

- **Excluye:**
  - Control desde Internet o redes externas (solo conexión local Bluetooth).
  - Control automático sin intervención de la mano (no hay algoritmo autónomo de estabilización).
  - Modelos dinámicos avanzados de la pelota (se trabaja de forma experimental).

## 4) Requisitos

- **Software**
  - Arduino IDE
  - Visual Studio Code (o editor equivalente)
  - Python 3.x
  - Librerías de Python:
    - `opencv-python`
    - `mediapipe`
    - `bluetooth` (PyBluez u otra implementación compatible)

- **Hardware**
  - ESP32
  - 3 servomotores
  - Protoboard
  - Cables jumper
  - Fuente de alimentación (por ejemplo 5 V para servos)
  - Cable USB para programar el ESP32
  - Estructura mecánica impresa en 3D (base, brazos, uniones)
  - Tornillos tipo M3, tuercas y rondanas

- **Requisitos de funcionamiento**
  - El ESP32 debe emparejarse correctamente por Bluetooth con la computadora.
  - El script de Python debe detectar la mano y generar ángulos dentro del rango 0°–180°.
  - El ESP32 debe recibir los comandos de texto y mover los servos con rampas suaves.
  - Ante la pérdida de comunicación, la plataforma debe regresar a una posición segura (HOME).

## 5) Diseño / Modelos 3D

### Base central
![Base](imgs/base.png)

### Brazo intermedio
![Brazo](imgs/brazo.png)

### Conector superior (“cuadrito”)
![Cuadrito](imgs/cuadrito.png)

### Horn del servo
![Horn del servo](imgs/hornservo.png)


## 6) Código fuente (Python + Arduino en un solo bloque)

```codigo
import cv2
import mediapipe as mp
import time
import bluetooth

# ================== CONEXIÓN BLUETOOTH ==================

PORT = 1
ESP32_MAC = "14:33:5C:02:4D:2A"   # CAMBIA a la MAC de tu ESP32

sock = bluetooth.BluetoothSocket()
sock.settimeout(20)

print("Intentando conectar al ESP32...")
while True:
    try:
        sock.connect((ESP32_MAC, PORT))
        print("¡Conectado al ESP32!")
        break
    except Exception as e:
        print("Error en conexión... reintentando:", e)
        time.sleep(1)

def send_bt(message: str):
    try:
        sock.send(message.encode())
        print("Enviado:", message.strip())
    except:
        print("Error enviando datos")


# ================== MEDIAPIPE ==================

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1,
                       min_detection_confidence=0.6,
                       min_tracking_confidence=0.5)
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)

# ===== FILTROS =====
pitch_filtrado = 0
roll_filtrado = 0
alpha = 0.25

ultimo_envio = time.time()
intervalo_envio = 0.05

# ===== HOME =====
HOME_IZQ = 90
HOME_ARRIBA = 90
HOME_DER = 90

# ===== GANANCIAS =====
K_pitch = 30.0
K_roll  = 0.05
K_lat   = 85.0
K_mid_acompa = 20.0


while cap.isOpened():

    ret, img = cap.read()
    if not ret:
        break

    img = cv2.flip(img, 1)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)
    h, w, _ = img.shape

    detectada = False

    if results.multi_hand_landmarks:
        for hand in results.multi_hand_landmarks:
            detectada = True
            mp_draw.draw_landmarks(img, hand, mp_hands.HAND_CONNECTIONS)

            # coordenadas importantes
            muneca = hand.landmark[0]
            medio  = hand.landmark[12]
            pulgar = hand.landmark[4]

            wx, wy = int(muneca.x*w), int(muneca.y*h)
            mx, my = int(medio.x*w),  int(medio.y*h)
            px, py = int(pulgar.x*w), int(pulgar.y*h)

            cv2.circle(img, (wx,wy), 10, (255,0,0), -1)
            cv2.circle(img, (mx,my), 10, (0,255,0), -1)
            cv2.circle(img, (px,py), 10, (0,0,255), -1)

            # ---------- cálculos ----------
            pitch = (muneca.z - medio.z) * 1.8
            pitch = max(-1, min(1, pitch))

            dy = wy - py
            roll = dy * K_roll
            roll = max(-1, min(1, roll))

            pitch_filtrado = (1-alpha)*pitch_filtrado + alpha*pitch
            roll_filtrado  = (1-alpha)*roll_filtrado  + alpha*roll

            if abs(pitch_filtrado) < 0.05:
                pitch_filtrado = 0
            if abs(roll_filtrado) < 0.05:
                roll_filtrado = 0

            # ---------- servos ----------
            # centro
            a_arriba = HOME_ARRIBA + K_pitch*pitch_filtrado + K_mid_acompa*abs(roll_filtrado)

            # laterales
            delta_lat = K_lat * roll_filtrado
            a_izq = HOME_IZQ - delta_lat
            a_der = HOME_DER + delta_lat

            # acompañamiento pitch
            a_izq += (K_pitch*0.25)*pitch_filtrado
            a_der += (K_pitch*0.25)*pitch_filtrado

            # límites
            a_izq = int(max(0, min(180, a_izq)))
            a_arriba = int(max(0, min(180, a_arriba)))
            a_der = int(max(0, min(180, a_der)))

            # enviar
            if time.time() - ultimo_envio >= intervalo_envio:
                msg = f"ANG:{a_izq},{a_arriba},{a_der}\n"
                send_bt(msg)
                ultimo_envio = time.time()

    if not detectada:
        cv2.putText(img, "No se detecta mano", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

    cv2.imshow("STEWART CONTROL PULGAR", img)

    k = cv2.waitKey(1)
    if k == ord('q'):
        break
    if k == ord('z') or k == ord('c'):
        send_bt("ZERO\n")

sock.close()
cap.release()
cv2.destroyAllWindows()
print("Programa terminado")

#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ===== Buffer para lectura BT no bloqueante =====
String btBuffer;

// ===== Pines de los servos =====
#define SERVO_IZQ    15   // Servo izquierdo
#define SERVO_ARRIBA 33   // Servo central / vertical
#define SERVO_DER    25   // Servo derecho

// ===== ÁNGULOS HOME =====
const int HOME_IZQ    = 90;
const int HOME_ARRIBA = 90;
const int HOME_DER    = 90;

// ===== PWM =====
const uint32_t FREQ_HZ   = 50;
const uint8_t  RES_BITS  = 12;
const uint16_t DUTY_MIN  = 205;   // ~1.0 ms (≈0°)
const uint16_t DUTY_MAX  = 410;   // ~2.0 ms (≈180°)

// Convierte grados físicos 0..180 a duty
uint16_t dutyFromDeg(int deg) {
  deg = constrain(deg, 0, 180);
  return map(deg, 0, 180, DUTY_MIN, DUTY_MAX);
}

// Convierte de ángulo lógico (0..180) a físico (invertido)
int logicalToPhysical(int logicalDeg) {
  logicalDeg = constrain(logicalDeg, 0, 180);
  return 180 - logicalDeg;  // invertido
}

// Escribe usando grados LÓGICOS
void writeServoLogical(int pin, int logicalDeg) {
  int fisico = logicalToPhysical(logicalDeg);
  ledcWrite(pin, dutyFromDeg(fisico));
}

// Configurar servo con ángulo lógico inicial
void configServo(int pin, int initialLogical) {
  pinMode(pin, OUTPUT);
  ledcAttach(pin, FREQ_HZ, RES_BITS);   // en core 3.x el pin ES el canal
  writeServoLogical(pin, initialLogical);
}

// ===== Rampa =====
const int  LIM_MIN     = 0;
const int  LIM_MAX     = 180;
const int  PASO_RAMPA  = 5;
const uint32_t DT_RAMP = 10;
const uint32_t TIMEOUT_MS = 700;

// Estado actual lógicos
int posIzq    = HOME_IZQ;
int posArriba = HOME_ARRIBA;
int posDer    = HOME_DER;

// Objetivos
int tgtIzq    = HOME_IZQ;
int tgtArriba = HOME_ARRIBA;
int tgtDer    = HOME_DER;

uint32_t tPrevRamp = 0;
uint32_t tLastCmd  = 0;

// Rampa suave
void aplicarRampa() {
  uint32_t now = millis();
  if (now - tPrevRamp < DT_RAMP) return;
  tPrevRamp = now;

  auto go = [&](int actual, int target){
    if(actual < target) return min(actual + PASO_RAMPA, target);
    if(actual > target) return max(actual - PASO_RAMPA, target);
    return actual;
  };

  posIzq    = go(posIzq,    tgtIzq);
  posArriba = go(posArriba, tgtArriba);
  posDer    = go(posDer,    tgtDer);

  writeServoLogical(SERVO_IZQ,    posIzq);
  writeServoLogical(SERVO_ARRIBA, posArriba);
  writeServoLogical(SERVO_DER,    posDer);
}

// Parsea "ANG:x,y,z"
bool parseAngulos(const String &msg, int &aIzq, int &aArriba, int &aDer){
  if(!msg.startsWith("ANG:")) return false;

  String data = msg.substring(4);
  int c1 = data.indexOf(',');
  int c2 = data.indexOf(',', c1+1);
  if(c1 < 0 || c2 < 0) return false;

  aIzq    = data.substring(0, c1).toInt();
  aArriba = data.substring(c1+1, c2).toInt();
  aDer    = data.substring(c2+1).toInt();

  return true;
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-Stewart");

  configServo(SERVO_IZQ,    posIzq);
  configServo(SERVO_ARRIBA, posArriba);
  configServo(SERVO_DER,    posDer);

  Serial.println("ESP32 listo - Plataforma Stewart");
  Serial.println("Pines: 15 = Izq, 33 = Arriba, 25 = Der");

  tLastCmd = millis();
}

void loop() {

  while(SerialBT.available()) {
    char c = (char)SerialBT.read();

    if(c == '\n') {
      String msg = btBuffer;
      btBuffer = "";
      msg.trim();

      if(msg.length() > 0) {
        tLastCmd = millis();

        if(msg == "ZERO" || msg == "LOST"){
          tgtIzq    = HOME_IZQ;
          tgtArriba = HOME_ARRIBA;
          tgtDer    = HOME_DER;
          Serial.println("HOME ejecutado (ZERO/LOST)");
        }
        else {
          int aI, aA, aD;
          if(parseAngulos(msg, aI, aA, aD)){
            tgtIzq    = constrain(aI, LIM_MIN, LIM_MAX);
            tgtArriba = constrain(aA, LIM_MIN, LIM_MAX);
            tgtDer    = constrain(aD, LIM_MIN, LIM_MAX);
            Serial.printf("ANG -> %d, %d, %d\n", tgtIzq, tgtArriba, tgtDer);
          }
          else {
            Serial.print("Comando desconocido: ");
            Serial.println(msg);
          }
        }
      }
    }
    else if(c != '\r') {
      btBuffer += c;
    }
  }

  if(millis() - tLastCmd > TIMEOUT_MS){
    tgtIzq    = HOME_IZQ;
    tgtArriba = HOME_ARRIBA;
    tgtDer    = HOME_DER;
  }

  aplicarRampa();
  delay(1);
}
```