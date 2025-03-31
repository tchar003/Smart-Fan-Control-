# Smart Fan Control

A temperature- and motion-responsive fan controller developed as a final project for the EE128 Embedded Systems course (Fall 2021).

This project uses a PWM signal from a K64F microcontroller to modulate fan speed based on ambient temperature, and includes a power-saving feature using a motion detector.

> **Note**: This project is no longer actively maintained or runnable due to lack of hardware. This repo serves as a portfolio reference.

---

## 🔧 Features
- **Automatic Fan Control** via ADC temperature sensor
- **Manual Mode Switching** using onboard switch (auto, speed 1–3)
- **Power-Saving Mode**: Turns off PWM after inactivity using PIR sensor
- **25kHz PWM Output** adjustable based on temperature
- **Visual Debugging**: PWM duty cycle shown via LED brightness

---

## 🧰 Hardware & Components
- NXP K64F Development Board
- MCP9700A-E/TO Temperature Sensor
- PIR Motion Sensor
- PWM-controlled Fan (demonstrated with LED)
- Resistors, Capacitors, Breadboard, Wires

---

## 🧠 System Design

- **Input**: Temperature Sensor (ADC) + PIR Motion Sensor (GPIO)
- **Control**: Interrupt-based state machine for manual mode switching
- **Output**: PWM signal (LED visualization)

![block diagram placeholder](images/system_diagram.png) <!-- Optional diagram -->

---

## 🚦 Testing
- Used breakpoints to monitor ADC values and PWM register updates
- Visualized PWM duty cycle changes via LED brightness
- Verified motion timeout behavior using sensor onboard LED

---

## 📜 Source Code
The core firmware logic can be found in [`src/`](src/) — written in C using the Kinetis SDK environment.

---

## 📽️ Demo Video
[![YouTube Demo](https://img.shields.io/badge/Watch%20Demo-YouTube-red?logo=youtube)](https://youtu.be/BLRwBStb1T0)

> Due to fan hardware issues and camera limitations, PWM behavior is visualized using LED cutofffs and not a smooth brightness curve. 

---

## 📄 Report
- Full PDF project summary available in [`docs/EE128_PROJECT.pdf`](docs/EE128_PROJECT.pdf)

---

## 📌 Notes
- PWM Fan hardware not available during testing
- LED used to demonstrate PWM output behavior

---

## 🧭 Future Improvements
- Improve calibration for temperature sensor
- Implement empirical fan curves
- Allow user adjustments to fan curve and power-saving delay
