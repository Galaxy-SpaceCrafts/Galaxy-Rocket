# 🚀 Galaxy Dragon | Advanced Flight Control System (GNC)

[![Version](https://img.shields.io/badge/Version-V1.5.0-blue.svg)](https://github.com/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://license/)
[![Stage](https://img.shields.io/badge/Stage-Active--Development-orange.svg)](https://status/)

**Galaxy Dragon** is a high-performance, autonomous rocket flight computer designed for suborbital missions. It implements advanced **Guidance, Navigation, and Control (GNC)** algorithms to ensure stability, apogee precision, and mission data integrity.

---

## 🛠 Technical Architecture

The system is built upon a dual-processor architecture, leveraging the **AVR ATmega2560** for real-time flight dynamics and the **ESP32-S** for high-speed telemetry and visual data acquisition.

### 🧠 Core Systems
* **Navigation (INS):** 6-DOF sensor fusion using an optimized **Kalman Filter** to mitigate barometric noise and IMU drift.
* **Control (PID):** Triple-axis stabilization with **Gain Scheduling** based on Mach number/velocity to prevent aeroelastic flutter.
* **Guidance:** GPS-based waypoint navigation and target safe-zone landing logic.
* **Actuation:** High-speed mix-logic for 3-fin 120° tail stabilization + 4-fin independent **Active Airbrakes**.

---

## 🛰 Version 1.5.0 Key Features

### 🌪 Active Drag Management (Airbrakes)
Implementation of a dynamic airbrake system that calculates real-time drag requirements to hit a precise target apogee. Includes a **Slew Rate Limiter** to protect servo integrity during high dynamic pressure (Max $Q$).

### 🎥 Synced Vision System
Integration with an **ESP32-CAM** co-processor:
* **Automated Triggering:** Recording starts exactly at $T+0$ (detected by $>20.0 m/s^2$ acceleration).
* **Fail-Safe Architecture:** Frame-by-frame JPEG storage to prevent data corruption during high-impact landings.

### 🛡 Mission Safety & Redundancy
* **Mach-Based Gain Scaling:** Automatically scales PID sensitivity as the rocket approaches transonic speeds.
* **Blackbox Logging:** Binary telemetry logging at 100Hz to a Class-10 SD Card.
* **Dynamic Abort Logic:** Auto-parachute deployment if the Pitch/Roll exceeds the **Abort Angle (45°)** during the boost phase.

---

## 📊 Flight Phases (State Machine)

1.  **PRE-FLIGHT:** System self-check, sensor calibration, and LoRa handshake.
2.  **BOOST:** Active stabilization and Mach-adaptive control.
3.  **COAST:** Airbrake deployment for apogee precision.
4.  **RECOVERY:** Dual-stage deployment (Drogue at Apogee, Main at 200m).
5.  **LANDING:** Data finalization and beacon transmission.

---

## 🔌 Hardware Configuration

| Component | Specification |
| :--- | :--- |
| **MCU** | Arduino Mega 2560 (16MHz AVR) |
| **Co-Processor** | ESP32-CAM AI-Thinker |
| **IMU** | MPU6050 (Accl/Gyro) |
| **Altimeter** | BMP280 (High Precision) |
| **Telemetry** | LoRa 433MHz (Long Range) |
| **GPS** | u-blox NEO-6M / TinyGPS++ |

---

> **Disclaimer:** This project involves high-energy propulsion systems. Always follow local safety regulations and NAR/Tripoli guidelines. Developed for educational and experimental aerospace research.
