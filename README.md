# Proyecto-Sistemas-Embebidos-MedMonitor-
# MedMonitor: Sistema de Telemetría Médica IoT 🩺

Monitor biométrico portátil basado en ESP32 para la medición de temperatura corporal, frecuencia cardíaca y saturación de oxígeno (SpO2) en tiempo real. 

## 📌 Características Principales
* **Adquisición No Invasiva:** Sensores MAX30102 (SpO2/BPM) y MLX90614 (Temperatura).
* **Edge Computing:** Servidor web local alojado en el propio ESP32 (modo Access Point).
* **Alertas Clínicas:** Clasificación de estado mediante un semáforo LED y notificaciones guiadas por voz (Módulo I2S MAX98357A).
* **Autonomía:** Operación continua >4 horas mediante batería LiPo y elevador Boost.

## 📂 Estructura del Repositorio
* `/Firmware`: Código fuente en C++ (PlatformIO / VS Code).
* `/Hardware`: Diagramas de conexión (Fritzing) y diseño de circuito impreso PCB (Proteus).
* `/Mechanics`: Modelos 3D de la carcasa (Archivo STL).

## 🛠️ Tecnologías y Herramientas
* **Microcontrolador:** ESP32 DevKit V1
* **Entorno de Desarrollo:** Visual Studio Code + PlatformIO
* **Diseño Electrónico:** Proteus ARES/ISIS & Fritzing

---
*Proyecto desarrollado para la materia de Sistemas Embebidos - ESPOL.*
