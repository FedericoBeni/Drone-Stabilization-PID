# Quadcopter PID Stabilization on STM32

This repository contains the firmware and documentation for a quadcopter stabilization project.

The primary goal of this project was to implement and tune a PID control system to maintain the balance of a stationary quadcopter. The drone is mounted on a test stand, and the control loop's objective is to keep it level (0° roll and pitch) by modulating the speed of its four motors.

## System Overview

The project involves both hardware integration and software development to achieve a stable, closed-loop control system. The firmware reads orientation data from an Inertial Measurement Unit (IMU) and uses a PID controller to compute the necessary adjustments, which are then sent to the motors as PWM signals.

### Key Hardware Components
- **Microcontroller:** [STM32H745 Nucleo-144](https://www.st.com/en/evaluation-tools/nucleo-h745zi-q.html) (Core of the system)
- **IMU Sensor:** [Adafruit BNO055](https://www.adafruit.com/product/2472) (9-DOF Absolute Orientation Sensor)
- **Motors:** 4x Turnigy D3536/9 910KV Brushless Motors
- **ESCs:** 4x Turnigy Plush-30A
- **RC Receiver:** Microzone MC6RE (Used as a safety switch to arm/disarm motors)

### Software & Control Logic
The firmware, developed using STM32CubeIDE, is built around a main control loop running at **100Hz**.

1.  **Sensing:** the BNO055 IMU provides real-time **roll** and **pitch** angles via I2C communication.
2.  **Error Calculation:** the system calculates the error between the current angle and the desired setpoint (0°).
3.  **PID Control:** two independent PID controllers (one for roll, one for pitch) process the error. After extensive testing, we determined that a **PD (Proportional-Derivative)** configuration provided the most stable results for our setup, with the integral term (I) set to zero.
4.  **Actuation:** the PID outputs are translated into target speeds for the four motors. These speeds are then converted into **PWM duty cycle** values and sent to the ESCs to control motor rotation.
5.  **Motor Arming:** a user button on the Nucleo board and a switch on the RC transmitter act as safety mechanisms to arm the motors before the control loop begins.

## Project Structure
The repository contains all the source code developed in STM32CubeIDE, including:
-   `main.c`: the main application loop and initialization routines.
-   `PID.c / PID.h`: implementation of the PID controller.
-   `bno055.c / bno055.h`: driver and interface functions for the IMU sensor.
-   `.ioc` file: STM32CubeMX configuration detailing pinouts, clock settings, and peripheral configurations (TIM, I2C, GPIO).

