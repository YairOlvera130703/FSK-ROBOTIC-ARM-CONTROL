# FSK-Controlled Robotic Arm 🤖📡

## Project Overview
This project demonstrates the remote control of a robotic arm utilizing **Frequency-Shift Keying (FSK) modulation**. Designed as a practical application of telecommunications and embedded systems, the system translates digital control commands into specific frequency signals, transmits them, and demodulates them at the receiver to execute precise physical movements.

This repository contains the logic architecture, microcontroller code, and hardware schematics required to achieve reliable FSK data transmission for robotic actuation.

## Technical Objectives
* Implement FSK modulation and demodulation for data transmission.
* Interface telecom hardware with microcontroller-based logic.
* Achieve stable, low-latency control of multi-axis robotic servos.

## Hardware & Software Stack
* **Microcontroller:** [Arduino UNO]
* **Modulation/Demodulation:** [XR2206/XR2211]
* **Actuators:** [MG996 SERVOMOTORS]
* **Programming Language:** C / C++ , LABVIEW
* **Development Environment:** [Arduino IDE]

## System Architecture
1. **Transmitter (Tx):** Reads user inputs and generates specific frequency tones (Mark and Space frequencies) representing binary states.
2. **Receiver (Rx):** Detects and demodulates the incoming audio frequencies back into binary logic.
3. **Control Unit:** Processes the decoded binary instructions to drive the robotic arm's corresponding joints.

---
*Developed by Oscar Yair Olvera Lopez*
