// PID_calculation.h

#ifndef PID_CALCULATION_H
#define PID_CALCULATION_H

#include <string.h>
#include <stdio.h>

// Define GPIO modes for pin configuration
#define OUTPUT GPIO_MODE_OUTPUT_PP  // Output mode for controlling motor direction
#define INPUT GPIO_MODE_INPUT       // Input mode for reading sensor data

// Variable to store sensor status as a bitfieldvoid readDigit(uint16_t *sensorValues, CalibrationData* calibration);
extern uint16_t sensorBits;
// Define white and black statesvoid Error_Handler(void);
extern uint8_t B;
extern uint8_t W;


// Function declarations for PID control and error calculation
void calculateError();      // Calculate error for PID control
void pidCalcul();           // Perform PID calculation
void resetPID();            // Reset PID controller values

// Function declaration to adjust motor speeds based on PID output
void ChangeMotorSpeed();    

// Main circulation control function
void circulation();         // Main function to control robot's movement

// Motor control functions to manage individual motor actions
void run_forward_1();       // Run motor 1 forward
void run_forward_2();       // Run motor 2 forward
void run_backward_1();      // Run motor 1 backward
void run_backward_2();      // Run motor 2 backward

// Functions to control the robot's overall movement
void stopBot();             // Stop the robot
void Frein();               // Apply brakes to the robot

void forward(uint16_t speed);      // Move robot forward at specified speed
void left(uint16_t speed);         // Turn robot left at specified speed
void sharpleft(uint16_t speed);    // Make a sharp left turn at specified speed

void right(uint16_t speed);        // Turn robot right at specified speed
void sharpright(uint16_t speed);   // Make a sharp right turn at specified speed

void reverse(uint16_t speed);      // Move robot backward at specified speed

#endif // PID_CALCULATION_H
