# STM32 PID Wheel Control System

This project implements a PID-based wheel control system for a differential drive robot using an STM32 microcontroller. The system uses ROS (Robot Operating System) for communication and control. This code was used for [MP_Bot](https://github.com/harshmahesheka/Multi-Purpose-HouseHold-Bot)

<p align="center">
  <img src="demo.gif" width="75%">
</p>

## Features

- PID-based motor control for precise wheel velocity control
- Differential drive kinematics implementation
- ROS integration for:
  - Wheel velocity control
  - Odometry publishing
  - Vacuum control
- Quadrature encoder support for accurate position tracking
- Configurable PID parameters

## Hardware Requirements

- STM32 microcontroller
- 2 DC motors with quadrature encoders
- Motor driver circuit
- Vacuum system (optional)
- ROS-compatible host computer

## Software Dependencies

- Arduino IDE
- ROS (Robot Operating System)
- STM32 Arduino Core
- ROS Serial library

## Pin Configuration

### Left Motor
- Encoder A: PA0
- Encoder B: PA1
- Motor P: PB5
- Motor N: PB4
- Motor Enable: PB3

### Right Motor
- Encoder A: PA4
- Encoder B: PA5
- Motor P: PA8
- Motor N: PC7
- Motor Enable: PB10

### Vacuum Control
- Vacuum PWM: PA7
