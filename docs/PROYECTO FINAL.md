# WiFi - Encender Leds

## 1) Resumen

- **Nombre del proyecto:** _Control de LEDs con ESP32 vía Servidor Web_  
- **Equipo / Autor(es):** _Jesus Cerezo - Karen Najera_  
- **Curso / Asignatura:** _Elementos programables II_  
- **Fecha:** _22/09/25_  
- **Descripción breve:** _Este proyecto utiliza un microcontrolador ESP32 conectado a WiFi para crear un servidor web que permite encender y apagar tres LEDs a través de una interfaz en el navegador. Demuestra los principios de redes, servidores HTTP y control de salidas digitales en sistemas embebidos._


## 2) Objetivos

- **General:** _Desarrollar un sistema de control remoto utilizando un ESP32 que, mediante un servidor web, permita encender y apagar múltiples LEDs desde cualquier dispositivo conectado a la misma red WiFi._
- **Específicos:**
  - _Configurar la conexión WiFi del ESP32 para establecer comunicación con la red y generar una dirección IP accesible desde un navegador_
  - _Implementar un servidor web en el ESP32 que interprete solicitudes HTTP y ejecute acciones sobre los pines digitales asociados a los LEDs._
  - _Diseñar una interfaz web sencilla que muestre el estado de los LEDs y permita controlarlos en tiempo real._

## 3) Alcance y Exclusiones

- Incluye
    * Controlar tres LEDs de forma remota mediante un servidor web alojado en el ESP32.
    * Demostrar la integración de red WiFi, servidor HTTP e interacción con hardware digital.
- Excluye
    - No se implementa interfaz gráfica avanzada, solo una página web básica.
    - No incluye control desde fuera de la red local

## 3) Requisitos


- **Software**
    * Arduino IDE
    * Navegador web en un dispositivo conectado a la misma red WiFi que el ESP32.
- **Hardware**
    - ESP32
    - Protoboard
    - 3 LEDS
    - 3 Resistencias
    - Cables jumper
    - Cable USB

-**Requisitos de funcionamiento)**
- El ESP32 debe conectarse correctamente a la red WiFi configurada.- El servidor web debe levantar en el puerto 80 y mostrar una página accesible desde la IP del ESP32.
- La interfaz web debe permitir encender y apagar los LEDs remotamente.
- El sistema debe reflejar el estado real de cada LED (ON/OFF) en la página.
- El control es local

**Conocimientos previos**
- _HTML básico_
- _Electrónica básica_
- _Programación en Arduino/ESP32_
