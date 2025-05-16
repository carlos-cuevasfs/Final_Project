# E.C.O.B.O.T. (Environmental Cleanup & Object-sorting BOT)

![MIT License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)  
![Maintained](https://img.shields.io/badge/status-maintained-brightgreen?style=for-the-badge)  
![ROS](https://img.shields.io/badge/ROS-22314E?style=for-the-badge&logo=ros&logoColor=white)  
![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)  
![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-C51A4A?style=for-the-badge&logo=raspberry-pi&logoColor=white)  
![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)  
![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)

---

## 📋 Prerequisites

- Basic knowledge of:
  - Digital electronics
  - Programming in Python and C++ (ROS and Arduino)
  - Computer vision fundamentals
  - CAD and 3D printing tools

- Required tools:
  - Multimeter
  - Soldering iron
  - 3D printer (FDM)
  - Precision screwdriver set
  - Laptop or PC with Ubuntu 20.04+

- Required components:
  - Arduino Uno or Mega
  - Raspberry Pi 4
  - MG996R Servomotors
  - 12V DC Motors
  - L293D Motor Driver
  - USB 1080p Camera
  - 12V 5000mAh Li-Ion Battery
  - Proximity, capacitive, and inductive sensors
  - 3D printed mechanical structure

- Required software:
  - Arduino IDE
  - ROS Noetic
  - OpenCV
  - Python 3
  - Fusion 360 or FreeCAD (optional)

---

## 📖 Introduction

E.C.O.B.O.T. is an autonomous robot designed to assist in waste management in urban public spaces, particularly parks. Its purpose is to detect, collect, and classify solid waste (metals, plastics, and cardboard) using computer vision, sensors, and autonomous navigation. Inspired by the UN Sustainable Development Goal 11, it aims to support cleaner and more sustainable cities through robotic automation.

---

## 🔩 Materials

| Component                        | Quantity |
|----------------------------------|----------|
| Raspberry Pi 4                   | 1        |
| Arduino Uno/Mega                 | 1        |
| USB Camera (1080p)               | 1        |
| 12V DC Motors                    | 4        |
| MG996R Servomotors               | 5        |
| L293D Motor Driver               | 1        |
| Proximity/Capacitive/Inductive Sensors | Several  |
| 12V 5000mAh Battery              | 1        |
| Caterpillar chassis              | 1        |
| Cables, connectors, protoboard   | Various  |
| 3D-Printed Structural Parts      | Several  |

---

## 💾 Software Installation

1. Install ROS Noetic on Ubuntu 20.04  
2. Clone this repository and compile using `catkin_make`  
3. Install dependencies:
   ```bash
   sudo apt-get install ros-noetic-usb-cam ros-noetic-cv-bridge python3-opencv
   pip install numpy imutils

## ⚙️ Montaje y Ensamblado

Pasos detallados para ensamblar el dispositivo, incluir diagramas y fotos del proceso

**Paso 1:** Ensamblar la base

**Paso 2:** Conectar los motores al microcontrolador

**Paso 3:** Asegurar los componentes en la carcasa

### 🔌 Conexiones Eléctricas

Diagrama esquemático y tabla de conexiones entre componentes:

---

## 💻 Programación

Código de ejemplo con explicación de cada parte relevante:

---

## ✅ Conclusión

Resumen de lo que se logró construir, aprendizajes obtenidos y posibles mejoras o versiones futuras del proyecto.

---

## 🔜 Mejoras futuras

- Enlistar las mejoras a realizar

## ⚠️ Advertencia

Como se indica en la licencia MIT, este software/hardware se proporciona **sin ningún tipo de garantía**. Por lo tanto, ningún colaborador es responsable de **cualquier daño a tus componentes, materiales, PC, etc..**.

---

## 📚 Recursos Adicionales

---

## 👥 Autores del proyecto

Autores originales del proyecto

---

## 📬 Contacto

¿Tienes dudas o sugerencias?

- 📧 Correo electrónico: ejemplo@udlap.mx

---
