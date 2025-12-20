# Incubadora Inteligente Agrícola 🐣 

Este proyecto consiste en una **incubadora automatizada** diseñada para optimizar y controlar los parámetros críticos de temperatura y humedad necesarios para la eclosión de huevos de gallina (ciclo de 21 días). Utiliza un sistema embebido basado en **STM32** con **FreeRTOS** para la gestión de tareas en tiempo real y comunicación remota vía **Telegram**.

Desarrollado para la cátedra de **Técnicas Digitales II (2025-R4051) - UTN FRBA**.

---

## 🚀 Características Principales

* **Control Ambiental Automático**: Regulación de temperatura (lámpara halógena) y humedad (humidificador ultrasónico) mediante lógica de histéresis.
* **Ciclo de Incubación Programado**: Gestión automática de las etapas críticas:
    * **Día 0-18 (Incubación)**: Temperatura ~37.7°C, Humedad ~55%, Rotación activa.
    * **Día 18-21 (Eclosión)**: Temperatura ~37.2°C, Humedad ~70%, Rotación detenida.
* **Rotación Automática**: Sistema de bandeja rotatoria por motor para asegurar el desarrollo correcto del embrión.
* **Interfaz de Usuario (HMI)**: Pantalla LCD 16x2 y encoder rotativo KY-040 para navegación de menús.
* **Alertas Remotas**: Notificaciones de alarmas enviadas a un bot de Telegram mediante un módulo ESP32-C3.
* **Persistencia de Datos**: El progreso se guarda en la memoria Flash para retomar el ciclo tras cortes de energía.

---

## 📸 Galería del Proyecto

<p align="center"><i>Vista general del prototipo.</i></p>
<p align="center">
  <img src="Documentacion/Fotos%20y%20videos/modelo_blender.png" width="60%" alt="Vista Superior Incubadora">
  <img src="Documentacion/Fotos%20y%20videos/WhatsApp Image 2025-12-18 at 14.07.39 (1).jpeg" width="31%" alt="Vista Superior Incubadora">
  <img src="Documentacion/Fotos%20y%20videos/WhatsApp Image 2025-12-18 at 14.07.39 (2).jpeg" width="30%" alt="Interior de la Incubadora">
  <img src="Documentacion/Fotos%20y%20videos/WhatsApp Image 2025-12-18 at 14.07.39 (3).jpeg" width="30%" alt="Vista Frontal de la incubadora">
  <img src="Documentacion/Fotos%20y%20videos/WhatsApp Image 2025-12-18 at 14.07.39.jpeg" width="30%" alt="Vista oblicua de la Incubadora">
</p>
<p align="center"><i>Vista del prototipo puesto a prueba.</i></p>

<p align="center">
  <img src="Documentacion/Fotos%20y%20videos/WhatsApp Image 2025-12-18 at 20.00.50.jpeg" width="50%" alt="Interfaz LCD">
  <img src="Documentacion/Fotos%20y%20videos/WhatsApp Image 2025-12-18 at 19.35.49.jpeg" width="50%" alt="Interfaz LCD">
</p>


---

## 🛠️ Hardware y Electrónica

El sistema central utiliza una placa **STM32 Nucleo-F446RE** conectada a un PCB diseñado en KiCad que gestiona tensiones de 3.3V, 5V, 12V y 220V.

### Componentes Clave:
* **Microcontroladores**: STM32F446RE y ESP32-C3 SuperMini (Bridge WiFi).
* **Sensores**: DHT11 para monitoreo ambiental.
* **Actuadores**: Módulo de 4 relés para lámpara, humidificador, cooler y motor.
* **Diseño de PCB**: Realizado íntegramente en **KiCad**.

---

## 💻 Arquitectura de Software (RTOS)

El firmware utiliza **FreeRTOS** para garantizar la ejecución concurrente de tareas críticas:

1.  **menuTask**: Gestión de la interfaz de usuario.
2.  **SensorTask**: Adquisición de datos del sensor DHT11.
3.  **ControlTask**: Lógica de control de temperatura y humedad.
4.  **MotorTask**: Control de tiempos de rotación.
5.  **Telegram/Bridge**: Comunicación serial con el ESP32 para alertas.

---

## 📁 Estructura del Repositorio

* `Incubadora_RTOS/`: Código fuente C para STM32 (STM32CubeIDE).
* `esp32c3-telegram/`: Firmware del módulo WiFi para Telegram.
* `Documentacion/`: Manuales de usuario, técnicos e informes.
* `PCB/`: Archivos de diseño electrónico (**KiCad**).
* `modelo_3D/`: Archivos de **Blender** y piezas **STL** para impresión.
* `Datasheets/`: Hojas de datos de los componentes.

---

## ⚙️ Configuración rápida

1.  **Carga de Firmware**: Flashear el proyecto de la carpeta `Incubadora_RTOS/` en la placa Nucleo.
2.  **Configuración WiFi**: Editar credenciales en `puenteTd2.ino` y cargar al ESP32.
3.  **Uso**: Iniciar el dispositivo, cargar agua en el tanque y seleccionar "INICIAR CICLO" desde el menú.

---

## 👥 Integrantes - Grupo 2
* **Carrettoni, Luciano**
* **Nanni, Franco**
* **Noé, Magdalena Cecilia**
* **Rios, Alexander Daniel**

**UTN FRBA - 2025**