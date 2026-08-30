# Joint ego noise reduction, source localization and beamforming for rescue vehicles

<div align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" alt="ROS2">
  <img src="https://img.shields.io/badge/Python-3.10-blue?logo=python" alt="Python">
  <img src="https://img.shields.io/badge/Arduino-C%2B%2B-00979D?logo=arduino" alt="Arduino">
  <img src="https://img.shields.io/badge/Hardware-Raspberry%20Pi-C51A4A?logo=raspberry-pi" alt="Raspberry Pi">
</div>

<div align="center">
  <a href="https://youtu.be/IhBLLOgKbUA">
    <img src="https://img.youtube.com/vi/IhBLLOgKbUA/0.jpg" alt="Sound-Tracking Rescue Robot Demo" width="70%">
  </a>
</div>
<br>

This repository contains the software and communication architecture for an autonomous search-and-rescue mobile platform. The system is designed to lock onto distress signals (human voices) using sound source localization, dynamically plan a path, and autonomously navigate to the target while avoiding obstacles.

## System Architecture

The robot's hardware and software are decoupled into three distinct modules, communicating via ROS2 and Serial interfaces:

*   **Sound Localization Module**: Utilizes two ReSpeaker microphone arrays to detect audio signals and calculate the Angle of Arrival (AOA).
*   **Sensing & Decision Module**: Powered by a Raspberry Pi integrated with a camera and an ultrasonic sensor for distance estimation and path planning.
*   **Motion Control Module**: Uses an Arduino UNO to drive DC motors equipped with encoders, processing commands to adjust the vehicle's attitude and speed.

## Key Technologies & Algorithms

*   **SRP-PHAT (Steered Response Power with Phase Transform)**: Implemented for multi-channel speech signal localization. It estimates the incident angle of the sound source by calculating phase differences across the microphone array, remaining highly robust in noisy environments.
*   **AOA (Angle of Arrival) Vector Intersection**: Fuses the directional data from both ReSpeaker arrays to calculate the exact coordinate distance and angle of the sound source.
*   **ROS2 Publish/Subscribe**: The Raspberry Pi runs multiple independent nodes (Audio processing, SRP-PHAT, AOA calculation) that share data in real-time via ROS2 topics, avoiding processing bottlenecks.
*   **Asynchronous Serial Communication**: The final trajectory data is transmitted from the Raspberry Pi to the Arduino via Serial Port.
*   **PID & Hardware Interrupts (ISR)**: The Arduino utilizes Timer and Interrupt Service Routines (ISR) to read AB-phase Hall encoders, applying a PID control loop every 50ms for precise motor rotation.

## Repository Structure

> **Note**: This repository contains a development snapshot of the project to demonstrate the core ROS2 integration and communication logic. 

*   `src/`
    *   `res_sub_and_pos_pub.py` - Core node for dual-microphone AOA data fusion and distance calculation.
    *   `arduino_reset.py` - Robust serial communication script handling handshake and hardware resets between Raspberry Pi and Arduino.
    *   `SRP-PHAT_Left.ipynb` / `SRP-PHAT_Right.ipynb` - DSP algorithms for left/right microphone arrays (Integrated into ROS2 nodes).
*   `experiments/` - Contains early development camera module tests and legacy communication prototypes.

## Team & Acknowledgements

This project was developed as a university capstone project at the Department of Power Mechanical Engineering, National Tsing Hua University, under the supervision of Prof. Ming-Sian Bai (白明憲). 

**Core Contributors:**
*   **Yuan-Fu Hung (洪元甫)**: Responsible for Acoustic Algorithms (SRP-PHAT/AOA), Embedded Systems (Arduino ISR/Timer), Raspberry Pi ROS2 Environment Setup, Hardware Communication, and System Integration.
*   **He-Sheng Liu (劉和昇)**: Responsible for Embedded Systems (Arduino ISR/Timer), Hardware Communication (Serial), and System Integration
*   **Chon-Hei Ng (吳俊希)**: Responsible for Mechanical Design, 3D Printing, Object Placement Planning, and System Integration.

*Original DSP algorithm baseline (SRP-PHAT) provided by lab seniors; ROS2 node refactoring, multi-array data fusion, and Serial communication implemented by Yuan-Fu Hung.*