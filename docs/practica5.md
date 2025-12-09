# 📚PRACTICA 5 - Comunicación_BLE_Básica

---

## 1) Resumen

- *Equipo / Autor(es):* ANGELES_BARRETO_EMMANUEL, CEREZO_PONCE_JESUS_ALFREDO  
- *Curso / Asignatura:* ELEMENTOS_PROGRAMABLES_II  
- *Descripción breve:* El programa configura un ESP32 como dispositivo Bluetooth Low Energy (BLE) que anuncia un servicio y una característica; permite comunicación básica con clientes BLE (smartphone, PC) para lectura/escritura de una cadena de texto.


---

## 2) Objetivos

- *General:* Implementar la comunicación inalámbrica mediante *Bluetooth Low Energy (BLE)* en el ESP32, comprendiendo el proceso de creación de un servidor BLE y su interacción con dispositivos clientes.  

- *Específicos:*  
  - Configurar el ESP32 como servidor BLE con un servicio personalizado.  
  - Crear una característica BLE con permisos de lectura y escritura.  
  - Analizar el funcionamiento del modo *advertising* y su detección por otros dispositivos.  
  - Comprobar la comunicación entre el ESP32 y una aplicación móvil BLE. 


## 3) Alcance y Exclusiones

- *Incluye:* El desarrollo de un servidor BLE funcional utilizando el ESP32, capaz de anunciar un servicio y permitir la lectura y escritura de datos desde una aplicación cliente BLE (como nRF Connect o LightBlue).  

- *No incluye:* La implementación de múltiples servicios simultáneos, encriptación de datos BLE, ni la integración con WiFi, Internet o plataformas IoT externas.  


---


## 4) Requisitos

*Software*  
- Arduino IDE versión 2.0 o superior.  
- Extensión de tarjetas ESP32 instalada desde el Gestor de Tarjetas.  
- Librerías necesarias: BLEDevice.h, BLEUtils.h y BLEServer.h.  
- Aplicación móvil para pruebas BLE (nRF Connect, LightBlue, o similar).  

*Hardware*  
- Microcontrolador ESP32 DevKit v1.  
- Cable USB para conexión y carga del programa.  
- Computadora con sistema operativo Windows, Linux o macOS.  

*Conocimientos previos*  
- Programación básica en C++ y entorno Arduino.  
- Conceptos de comunicación inalámbrica (Bluetooth/BLE).  
- Uso de herramientas de monitoreo serial y escaneo BLE.  
---

## 5) Instalación / Código ⌨

bash
# ========================================
# LIBRERÍAS NECESARIAS
# ========================================
include <BLEDevice.h>
include <BLEUtils.h>
include <BLEServer.h>

# ========================================
# CONFIGURACIÓN PRINCIPAL (versión ajustada)
# ========================================
static const char* DEVICE_NAME = "AK_BLE_Node";  # Nombre visible al escanear
define SERVICE_UUID_NEW        "87654321-4321-4321-4321-0987654321ab"
define CHARACTERISTIC_UUID_NEW "dcba4321-8765-09ab-cdef-0123456789ab"

# ========================================
# CONFIGURACIÓN INICIAL (setup)
# ========================================
void setup() {
  Serial.begin(115200);
  delay(120);  # pequeño retardo para estabilizar el puerto serie

  # Inicializa BLE con el nombre definido arriba
  BLEDevice::init(DEVICE_NAME);

  # Crea servidor BLE y agrega un servicio con UUID personalizado
  BLEServer* pServer = BLEDevice::createServer();
  BLEService* pService = pServer->createService(SERVICE_UUID_NEW);

  # Crea una característica con permisos de lectura y escritura
  BLECharacteristic* pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_NEW,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );

  # Valor inicial de la característica (puedes cambiarlo desde un cliente BLE)
  pCharacteristic->setValue("Hola mundo BLE");

  # Arranca el servicio para hacerlo disponible
  pService->start();

  # Configura el advertising para que el dispositivo sea detectable
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID_NEW);   # Anuncia el UUID del servicio
  pAdvertising->setScanResponse(true);              # Incluir respuesta de escaneo
  pAdvertising->setMinPreferred(0x06);              # Parámetros recomendados por el core
  pAdvertising->setMinPreferred(0x12);

  # Inicia la publicidad BLE (advertising)
  BLEDevice::startAdvertising();
  Serial.println("BLE Advertising iniciado (AK_BLE_Node)");
}

# ========================================
# BUCLE PRINCIPAL (loop)
# ========================================
void loop() {
  # El advertising funciona en background; no es necesario hacer nada aquí
  delay(1000);
}


## 5) Conclusión 🧠

>En esta práctica se logró implementar un *servidor Bluetooth Low Energy (BLE)* en el ESP32, comprendiendo el proceso de inicialización, creación de servicios y características, así como la transmisión de información mediante el modo *advertising*.  
>Se verificó la conexión con dispositivos externos y la capacidad del microcontrolador para actuar como un *servidor BLE estable y de bajo consumo, lo cual es esencial en aplicaciones de **IoT, monitoreo inalámbrico y sistemas portátiles inteligentes*.  
>Esta práctica refuerza los fundamentos de la comunicación inalámbrica moderna y sienta las bases para proyectos más complejos que integren BLE con sensores y plataformas móviles.