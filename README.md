# QTR-RC Sensor Driver(STM32F4) and PID Implementation

This repository provides an implementation of a custom **QTR-RC sensor driver** for the **STM32F4** microcontroller and integrates a **PID (Proportional-Integral-Derivative) controller** to manage motor movement based on sensor data. This project is noteworthy as it introduces an QTR-RC sensor driver compatible with STM32F4, which is currently unavailable elsewhere.

## Project Structure

The repository is organized into the following folders:

1. **QTR_RC_Driver**
   - Contains the implementation of the QTR-RC sensor driver for STM32F4, Successfully tested on STM32F446RE Board.
   - This folder includes essential files for configuring GPIO pins, reading sensor values, and calibrating the sensors.

2. **PID_implementation For Line-follower**
   - Contains the PID controller code responsible for adjusting motor speeds based on sensor input to maintain desired movement patterns.
   - The PID controller calculates error calculations based on sensor feedback, which requires correction to keep the robot on track, enabling smooth and responsive navigation.
   - Includes functions to manage motor speed and directional adjustments based on PID output.

## Key Features

- **Custom QTR-RC Sensor Driver**: A newly developed driver for QTR-RC sensors, specifically tailored for the STM32F4 series.
- **PID-Controlled Motor Speed Adjustment for LineFollower Robot**: Real-time PID-based adjustments to motor speeds, allowing precise control over robot movement.
- **Modular Structure**: Organized into separate modules for sensor interfacing and control logic, making it easy to maintain and expand.

## File Descriptions

### qtr-rc_driver

- `qtr_rc_driver.h`: Header file defining functions and GPIO modes for QTR-RC sensor handling.
- `qtr_rc_driver.c`: Source file implementing the QTR-RC sensor reading, calibration, and error calculation logic.

### pid_implementation

- `PID_calculation.h`: Header file defining PID functions and parameters.
- `PID_calculation.c`: Source file implementing the PID control calculations and motor speed adjustments based on sensor feedback.

## Getting Started

To use this code with an STM32F4 board:
1. Clone the repository to your local machine.
   ```bash
   git clone https://github.com/WassimHedfi/QTR_RC-driver-STM32f4.git
