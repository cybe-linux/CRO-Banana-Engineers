# Autonomous Robot Vehicle Control & Navigation System

## Overview
This repository contains the software architecture, controller firmwares, and hardware interface scripts for an autonomous robot car chassis. The vehicle utilizes a hybrid compute pipeline consisting of a Raspberry Pi main controller (handling high-level decision making, navigation logic, and high-speed motor control) and an Arduino co-processor (dedicated to real-time low-latency sensor acquisition).

The physical vehicle platform utilizes a belt-driven rear wheel drive and a front servo-based steering.

---

## 1. System Architecture & Electromechanical Integration

The solution decouples high-level autonomous navigation from low-level sensor polling. Communication between the main processor and the sensor co-processor is maintained over a high-speed USB Serial protocol (115200 baud).

## Module Breakdown & Electromechanical Mapping

* Main Processing Unit: Raspberry Pi
  * Language/Framework: Python 3 with numpy, pyserial, and opencv libraries.

  * Role: Runs the primary control loop, processing telemetry data, reading IMU orientation data directly over the I2C bus, and generating real-time driving PWM signals.

  * Electromechanical Linkage:

    * Drive Motor / ESC: Connected via GPIO Pin 12 (PWM) to the Electronic Speed Controller (ESC). The ESC drives the mid-mounted 12V DC motor, which transmits torque via a timing belt to the solid rear axle.

    * Steering Servo: Connected via GPIO Pin 13 (PWM) directly to the front steering servo horn. The servo adjusts front tie-rods to control vehicle direction via Ackermann geometry.

    * IMU (Inertial Measurement Unit): Connected directly to GPIO Pin 2 (SDA) and GPIO Pin 3 (SCL) to track vehicle heading, tilt, and acceleration.

* Co-Processing Unit: Arduino
  * Language/Framework: C++ / Arduino Framework.

  * Role: Offloads high-frequency analog and digital sensor reading (such as ultrasonic distance sensors, wheel encoders, and infrared rangefinders) from the Raspberry Pi's operating system. It streams formatted, comma-separated telemetry frames over USB Serial to ensure zero CPU starvation on the main navigation thread.

  * Electromechanical Linkage: Connected directly to physical rangefinding sensors mounted on the front and sides of the chassis, and connected to the Raspberry Pi via standard USB cable for data transfer and logic power.
