# 📚 Práctica 2 - ComunicaciónSerial_NeoPixel

> El programa controla un LED NeoPixel mediante comunicación serial (Serial.begin(115200)), encendiéndolo en color magenta (RGB: 122, 0, 122) únicamente cuando recibe un mensaje (if Serial.available() > 0 y Serial.readStringUntil('\n')). Ocupamos conceptos de control de LEDs RGB con pixels.setPixelColor() y pixels.show(), lectura de datos seriales y condicionales. Es ideal para aprender la interacción entre Arduino y dispositivos externos mediante el puerto serial, permitiendo activar el LED bajo demanda.


---

## 1) Resumén 📌

- *Equipo / Autor(es):* Emmanuel Angeles Barreto
- *Curso / Asignatura:* Elementos Programables II  
- *Fecha:* 23/09/25  

---

## 2) Código ⌨

bash
# Desde el primer include hasta el ultimo "define" llevan #
# ========================================
# CÓDIGO 2: CONTROL DE LED NEOPIXEL POR COMUNICACIÓN SERIAL
# ========================================

# Desde el primer include hasta el ultimo "define" llevan #
include <Adafruit_NeoPixel.h>  // Librería para controlar LEDs NeoPixel RGB direccionables
ifdef __AVR__
   include <avr/power.h>  // Librería específica para placas AVR (optimización de energía)
endif

# ========================================
# CONFIGURACIÓN DE CONSTANTES
# ========================================
define PIN  8          // Pin digital del ESP32 donde está conectado el LED NeoPixel
define NUMPIXELS 1     // Cantidad total de LEDs en la tira (en este caso solo 1)

# ========================================
# INICIALIZACIÓN DEL OBJETO NEOPIXEL
# ========================================
# Parámetros: cantidad de LEDs, pin de datos, tipo de LED (GRB) + frecuencia (800KHz)
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

# ========================================
# CONFIGURACIÓN INICIAL (se ejecuta una sola vez)
# ========================================
void setup(){
  Serial.begin(115200);  // Inicializar comunicación serial a 115200 baudios para recibir comandos
  pixels.begin();        // Inicializar el sistema NeoPixel (configurar pin como salida)
}

# ========================================
# BUCLE PRINCIPAL (se ejecuta infinitamente)
# ========================================
void loop(){
  # Paso 1: Verificar si hay datos disponibles en el puerto serial
  if (Serial.available() > 0){
    
    # Paso 2: Leer el mensaje recibido hasta encontrar un salto de línea ('\n')
    String msj = Serial.readStringUntil('\n');
    
    # Paso 3: Mostrar el mensaje recibido en el Serial Monitor (retroalimentación)
    Serial.println(msj);
    
    # Paso 4: Apagar el LED antes de configurar el nuevo color
    pixels.clear();
    
    # Paso 5: Configurar el color del LED en índice 0
    # Color morado/magenta: Rojo=122, Verde=0, Azul=122
    # Valores van de 0 (apagado) a 255 (máximo brillo)
    pixels.setPixelColor(0, pixels.Color(122, 0, 122));
    
    # Paso 6: Enviar los datos al LED para que muestre el color configurado
    pixels.show();
  }
  
  # Paso 7: Esperar 700 milisegundos (0.7 segundos) antes de la siguiente iteración
  # Este delay se ejecuta siempre, haya o no mensaje recibido
  delay(700);
  
  # IMPORTANTE: El LED solo se enciende cuando se recibe un mensaje por el puerto serial
  # Si no hay mensajes, el LED permanece en su último estado
}

---

# 3) Evidencias del resultado final 📸

![Diagrama del sistema](recursos/imgs/ledcolor.jpg)

[VIDEO MUESTRA YT][doc-ref]

[doc-ref]: https://youtu.be/20FBP9kdGKE "Video muestra YT"


---