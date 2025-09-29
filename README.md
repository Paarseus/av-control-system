# Autonomous Vehicle Control System

## Overview

This repository contains embedded firmware for a distributed drive-by-wire control system developed at the Autonomous Vehicle Research Lab at Cal Poly Pomona. The system implements real-time control of steering, throttle, and braking subsystems through a CAN bus architecture using Teensy 4.1 microcontrollers, with UDP-based telemetry and camera streaming integration for remote monitoring via SimChair.

## System Architecture

The control system consists of three independent microcontroller nodes communicating over a 250kbps CAN bus:

- **Steering Module**: Stepper motor control with analog position feedback and limit switch safety
- **Throttle Module**: Multi-channel DAC (MCP4728) interface for throttle position and drive mode selection (Neutral/Drive/Sport/Reverse)
- **Brake Module**: Linear actuator control with position feedback (referenced in master test code)

Each subsystem operates autonomously while responding to commands from a central controller, enabling modular development and fault isolation.

### Remote Monitoring & Control

- **UDP telemetry interface** for real-time vehicle state transmission
- **Camera streaming** to SimChair platform for visual feedback
- **Bidirectional communication** enabling remote control and data visualization

## Key Features

- **Real-time CAN communication** with standardized message IDs and protocols
- **Safety mechanisms**: Hardware limit switches, watchdog timers, emergency stop capability
- **Closed-loop position control** using analog feedback sensors
- **Mode sequencing** for safe state transitions
- **Modular architecture** allowing independent subsystem testing and integration
- **Network telemetry** with UDP streaming for remote monitoring and control
- **Video feedback** integrated with SimChair interface

## Hardware

- Teensy 4.1 microcontrollers (ARM Cortex-M7 @ 600 MHz)
- MCP4728 quad-channel 12-bit DAC
- Stepper motor drivers
- Linear actuators with analog position feedback
- CAN transceivers
- Network-enabled camera system

## Development Status

This is active research code developed for experimental autonomous vehicle testing. All testing is conducted in controlled laboratory environments with appropriate safety protocols.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

