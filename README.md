# COMP50069 – IoT-Based Embedded System
## Scenario 1: The Secure Industrial Loading Gate

**Module:** Hardware, Microcontrollers and Sensors (COMP50069)
**Institution:** APIIT Sri Lanka | University of Staffordshire
**Student:** Dilnuka Vindi
**Student ID:** [Your Student ID]
**Date:** [Submission Date]

---

## Project Overview

This repository contains the complete firmware and supporting materials for an IoT-based embedded system that autonomously manages an industrial loading gate using an ESP32 microcontroller.

The system implements:
- **Autonomous mode** — PIR-triggered gate opening with timed auto-close
- **Manual mode** — button-controlled override locking the gate open
- **Safety logic** — ultrasonic obstruction detection (< 20 cm) with immediate halt
- **Option D Hybrid recovery** — transient auto-recovery + 3-strike escalation to permanent freeze
- **Live OLED display** and UART command interface

---

## Repository Structure
COMP50069-IoT-Scenario1-LoadingGate/
├── README.md # This file
├── src/
│ └── main.ino # ESP32 firmware (Doxygen-documented)
├── docs/
│ └── diagrams/ # State machine, architecture, wiring diagrams
└── images/
└── hardware/ # Photos of physical prototype

---

## Hardware Requirements

| Component | Quantity |
|---|---|
| ESP32 DevKit (ESP32-32U) | 1 |
| PIR motion sensor (HC-SR501) | 1 |
| Ultrasonic sensor (HC-SR04) | 1 |
| 10 kΩ potentiometer | 1 |
| SG90 micro-servo | 1 |
| SSD1306 I²C OLED (128×64) | 1 |
| Active buzzer (5 V) | 1 |
| Tactile push buttons | 2 |
| LEDs (green, red) | 2 |
| 220 Ω resistors | 2 |
| 1 kΩ + 2 kΩ resistors (voltage divider) | 1 set |
| Breadboard + jumper wires | — |

---

## Pin Assignment

| ESP32 Pin | Peripheral |
|---|---|
| GPIO 4 | PIR sensor OUT |
| GPIO 5 | Ultrasonic TRIG |
| GPIO 18 | Ultrasonic ECHO (via 1 kΩ/2 kΩ divider) |
| GPIO 34 | Potentiometer wiper |
| GPIO 13 | Servo signal |
| GPIO 26 | Buzzer |
| GPIO 32 | Green LED |
| GPIO 33 | Red LED |
| GPIO 27 | Error Reset button |
| GPIO 14 | Manual Mode button |
| GPIO 21 | OLED SDA |
| GPIO 22 | OLED SCL |

---

## Live Simulation

**Wokwi:** https://wokwi.com/projects/475982909567509505

The simulation includes all 11 components wired as described in the report, with the full firmware preloaded.

---

## Dependencies

- `Wire.h` (built-in)
- `Adafruit_GFX` (Library Manager)
- `Adafruit_SSD1306` (Library Manager)
- `ESP32Servo` (Library Manager)

---

## Building and Flashing

1. Install the Arduino IDE (2.x recommended).
2. Add ESP32 board support via Boards Manager: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. Install the three libraries listed above.
4. Open `src/main.ino`.
5. Select board: **ESP32 Dev Module**.
6. Select the correct COM port.
7. Click **Upload**.

---

## Operating Modes

| Mode | Entered By | Behaviour |
|---|---|---|
| AUTONOMOUS | Default / recovery | PIR-driven gate open, timed auto-close |
| MANUAL | Manual button | Gate locked open, automation suspended |
| SAFETY_HALT | Ultrasonic < 20 cm | Gate frozen, buzzer alarm, OLED alert |
| TIMED_IDLE | Obstruction cleared | 3-second wait before auto-retreat |
| PERMANENT_FREEZE | 3 consecutive events | Locked until Error Reset pressed |

---

## Licence

This project was developed for academic submission to COMP50069. No commercial licence is granted.