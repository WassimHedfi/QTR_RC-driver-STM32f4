// Variables for PID control
double error;
double previous_error;

double output;
double integral, derivative;

// PID tuning parameters
double kp = 200;      // Proportional gain
double ki = 0.017;    // Integral gain
double kd = 550;      // Derivative gain

// Terms for the PID calculation
double pTerm = 0.0;
double iTerm = 0.0;
double dTerm = 0.0;

// Motor speed variables
int16_t motor1newSpeed;      // New speed for motor 1
int16_t motor2newSpeed;      // New speed for motor 2
int16_t motor2Speed = 950;   // Initial speed for motor 2
int16_t motor1Speed = 950;   // Initial speed for motor 1
int16_t MAX_MOTOR_SPEED = 1000; // Maximum speed for both motors

uint16_t back = 400;        // Base backward speed
float a = 0.5;              // Adjustment factor
uint16_t back_speed = 400;  // Backward speed for motors
uint16_t error_back = 10;   // Error threshold for backward adjustment



// Reset PID variables to initial state
void resetPID() {
    integral = 0;
    error = 0;
    previous_error = 0;
}

// Perform PID calculations for motor control
void pidCalcul() {
    pTerm = kp * error;                // Calculate proportional term
    integral += error;                 // Accumulate error for integral term
    iTerm = ki * integral;             // Calculate integral term
    derivative = error - previous_error; // Calculate derivative
    dTerm = kd * derivative;           // Calculate derivative term
    output = pTerm + iTerm + dTerm;    // Calculate total PID output

    previous_error = error;            // Update previous error
}

// Calculate the error based on sensor readings
void calculateError() {
    // Combine the 16 IR sensor readings into a single 16-bit integer
	sensorBits = 0;
    for (int i = 16; i > 0; i--) {
        sensorBits |= (ir[i-1] == 1) << (i-1);
    }




    // Define masks for detecting sensor patterns and calculating errors
    switch(sensorBits) {
    	case 0x0001: // 0000 0000 0000 0001
    		error = -7.5;
    		break;
    	case 0x0003: // 0000 0000 0000 0011
    		error = -7;
    		break;
    	case 0x0007: // 0000 0000 0000 0111
    		error = -6.5;
    		break;
    	case 0x000F: // 0000 0000 0000 1111
    		error = -6;
    		break;
    	case 0x001F: // 0000 0000 0001 1111
    		error = -5.5;
    		break;
    	case 0x001E: // 0000 0000 0001 1110
    		error = -5;
    		break;
    	case 0x003E: // 0000 0000 0011 1110
    		error = -4.5;
    		break;
    	case 0x003C: // 0000 0000 0011 1100
    		error = -4;
    		break;
    	case 0x007C: // 0000 0000 0111 1100
    		error = -3.5;
    		break;
    	case 0x0078: // 0000 0000 0111 1000
    		error = -3;
    		break;
    	case 0x00F8: // 0000 0000 1111 1000
    		error = -2.5;
    		break;
    	case 0x00F0: // 0000 0000 1111 0000
    		error = -2;
    		break;
    	case 0x01F0: // 0000 0001 1111 0000
    		error = -1.5;
    		break;
    	case 0x01E0: // 0000 0001 1110 0000
    		error = -1;
    		break;
    	case 0x03E0: // 0000 0011 1110 0000
    		error = -0.5;
    		break;
    	case 0x03C0: // 0000 0011 1100 0000
    		error = 0;
    		break;
    	case 0x07C0: // 0000 0111 1100 0000
    		error = 0.5;
    		break;
    	case 0x0780: // 0000 0111 1000 0000
    		error = 1;
    		break;
    	case 0x0F80: // 0000 1111 1000 0000
    		error = 1.5;
    		break;
    	case 0x0F00: // 0000 1111 0000 0000
    		error = 2;
    		break;
    	case 0x1F00: // 0001 1111 0000 0000
    		error = 2.5;
    		break;
    	case 0x1E00: // 0001 1110 0000 0000
    		error = 3;
    		break;
    	case 0x3E00: // 0011 1110 0000 0000
    		error = 3.5;
    		break;
    	case 0x3C00: // 0011 1100 0000 0000
    		error = 4;
    		break;
    	case 0x7C00: // 0111 1100 0000 0000
    		error = 4.5;
    		break;
    	case 0x7800: // 0111 1000 0000 0000
    		error = 5;
    		break;
    	case 0xF800: // 1111 1000 0000 0000
    		error = 5.5;
    		break;
    	case 0xF000: // 1111 0000 0000 0000
    		error = 6;
    		break;
    	case 0xE000: // 1110 0000 0000 0000
    		error = 6.5;
    		break;
		case 0xC000: // 1100 0000 0000 0000
			error = 7;
			break;
		case 0x8000: // 1000 0000 0000 0000
			error = 7.5;
			break;


		// _______________________________________________twos & threes___________________________________________________

	    case 0x0006: // 0000 0000 0000 0110
	        error = -6;
	        break;
	    case 0x000E: // 0000 0000 0000 1110
	        error = -5.5;
	        break;
	    case 0x000C: // 0000 0000 0000 1100
	        error = -5;
	        break;
	    case 0x001C: // 0000 0000 0001 1100
	        error = -4.5;
	        break;
	    case 0x0018: // 0000 0000 0001 1000
	        error = -4;
	        break;
	    case 0x0038: // 0000 0000 0011 1000
	        error = -3.5;
	        break;
	    case 0x0030: // 0000 0000 0011 0000
	        error = -3;
	        break;
	    case 0x0070: // 0000 0000 0111 0000
	        error = -2.5;
	        break;
	    case 0x0060: // 0000 0000 0110 0000
	        error = -2;
	        break;
	    case 0x00E0: // 0000 0000 1110 0000
	        error = -1.5;
	        break;
	    case 0x00C0: // 0000 0000 1100 0000
	        error = -1;
	        break;
	    case 0x01C0: // 0000 0001 1100 0000
	        error = -0.5;
	        break;
	    case 0x0180: // 0000 0001 1000 0000
	        error = 0;
	        break;
	    case 0x0380: // 0000 0011 1000 0000
	        error = 0.5;
	        break;
	    case 0x0300: // 0000 0011 0000 0000
	        error = 1;
	        break;
	    case 0x0700: // 0000 0111 0000 0000
	        error = 1.5;
	        break;
	    case 0x0600: // 0000 0110 0000 0000
	        error = 2;
	        break;
	    case 0x0E00: // 0000 1110 0000 0000
	        error = 2.5;
	        break;
	    case 0x0C00: // 0000 1100 0000 0000
	        error = 3;
	        break;
	    case 0x1C00: // 0001 1100 0000 0000
	        error = 3.5;
	        break;
	    case 0x1800: // 0001 1000 0000 0000
	        error = 4;
	        break;
	    case 0x3800: // 0011 1000 0000 0000
	        error = 4.5;
	        break;
	    case 0x3000: // 0011 0000 0000 0000
	        error = 5;
	        break;
	    case 0x7000: // 0111 0000 0000 0000
	        error = 5.5;
	        break;
	    case 0x6000: // 0110 0000 0000 0000
	        error = 6;
	        break;






		case 0x0000: // 0000 0000 0000 0000
			if (activate_error_back == 1){

				if (previous_error < -3) {

					error = -error_back;

				} else if (previous_error > 3) {

					error = error_back;

				}
			}

			break;

		default:
			break; // Default case when no matching pattern
	}
}


// Main circulation control function
void circulation() {
    // Read sensor values and calibrate based on reference
    readDigit(sensorValues, &calibration);

    // Calculate error based on the sensor readings for PID adjustment
    calculateError();

    // Execute PID calculations to determine output for motor speed adjustment
    pidCalcul();

    // Adjust motor speeds based on calculated PID output
    ChangeMotorSpeed();
}

// Adjust motor speeds based on PID output
void ChangeMotorSpeed() {
    // Calculate new speeds for each motor by applying the PID output adjustment
    motor1newSpeed = motor1Speed - output;
    motor2newSpeed = motor2Speed + output;

    // Adjust speed for motor 1
    if (motor1newSpeed >= 0) {
        // Ensure motor 1 speed does not exceed maximum limit
        if (motor1newSpeed > MAX_MOTOR_SPEED) motor1newSpeed = MAX_MOTOR_SPEED;
        run_forward_1(); // Move motor 1 forward
    } else if (motor1newSpeed < -back) {
        // Handle negative speed by scaling with factor `a` and limiting to maximum speed
        motor1newSpeed = back + a * (-motor1newSpeed);
        if (motor1newSpeed > MAX_MOTOR_SPEED) motor1newSpeed = MAX_MOTOR_SPEED;
        run_backward_1(); // Move motor 1 backward
    } else {
        // Set motor 1 speed to zero (stop or run forward with zero speed)
        motor1newSpeed = 0;
        run_forward_1();
    }

    // Adjust speed for motor 2
    if (motor2newSpeed >= 0) {
        // Ensure motor 2 speed does not exceed maximum limit
        if (motor2newSpeed > MAX_MOTOR_SPEED) motor2newSpeed = MAX_MOTOR_SPEED;
        run_forward_2(); // Move motor 2 forward
    } else if (motor2newSpeed < -back) {
        // Handle negative speed by scaling with factor `a` and limiting to maximum speed
        motor2newSpeed = back + a * (-motor2newSpeed);
        if (motor2newSpeed > MAX_MOTOR_SPEED) motor2newSpeed = MAX_MOTOR_SPEED;
        run_backward_2(); // Move motor 2 backward
    } else {
        // Set motor 2 speed to zero (stop or run forward with zero speed)
        motor2newSpeed = 0;
        run_forward_2();
    }
}





// Motor control functions for movement directions
void run_forward_1() {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, motor1newSpeed);  // Set speed for motor 1
    HAL_GPIO_WritePin(motor1Forward_GPIO_Port, motor1Forward_Pin, GPIO_PIN_SET);  // Run motor 1 forward
    HAL_GPIO_WritePin(motor1Backward_GPIO_Port, motor1Backward_Pin, GPIO_PIN_RESET);  // Stop motor 1 backward
}

void run_forward_2() {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, motor2newSpeed);  // Set speed for motor 2
    HAL_GPIO_WritePin(motor2Forward_GPIO_Port, motor2Forward_Pin, GPIO_PIN_SET);  // Run motor 2 forward
    HAL_GPIO_WritePin(motor2Backward_GPIO_Port, motor2Backward_Pin, GPIO_PIN_RESET);  // Stop motor 2 backward
}

void run_backward_1() {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, motor1newSpeed);  // Set speed for motor 1
    HAL_GPIO_WritePin(motor1Forward_GPIO_Port, motor1Forward_Pin, GPIO_PIN_RESET);  // Stop motor 1 forward
    HAL_GPIO_WritePin(motor1Backward_GPIO_Port, motor1Backward_Pin, GPIO_PIN_SET);  // Run motor 1 backward
}

void run_backward_2() {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, motor2newSpeed);  // Set speed for motor 2
    HAL_GPIO_WritePin(motor2Forward_GPIO_Port, motor2Forward_Pin, GPIO_PIN_RESET);  // Stop motor 2 forward
    HAL_GPIO_WritePin(motor2Backward_GPIO_Port, motor2Backward_Pin, GPIO_PIN_SET);  // Run motor 2 backward
}

void forward(uint16_t speed){
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);  // Change motor 2 speed
	  HAL_GPIO_WritePin(motor1Forward_GPIO_Port, motor1Forward_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(motor1Backward_GPIO_Port, motor1Backward_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(motor2Forward_GPIO_Port, motor2Forward_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(motor2Backward_GPIO_Port, motor2Backward_Pin, GPIO_PIN_RESET);

}


void sharpleft(uint16_t speed){
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);  // Change motor 2 speed
	  HAL_GPIO_WritePin(motor1Forward_GPIO_Port, motor1Forward_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(motor1Backward_GPIO_Port, motor1Backward_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(motor2Forward_GPIO_Port, motor2Forward_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(motor2Backward_GPIO_Port, motor2Backward_Pin, GPIO_PIN_RESET);
}

void left(uint16_t speed){
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);  // Change motor 2 speed
	  HAL_GPIO_WritePin(motor1Forward_GPIO_Port, motor1Forward_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(motor1Backward_GPIO_Port, motor1Backward_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(motor2Forward_GPIO_Port, motor2Forward_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(motor2Backward_GPIO_Port, motor2Backward_Pin, GPIO_PIN_RESET);
}

void right(uint16_t speed){
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);  // Change motor 2 speed
	  HAL_GPIO_WritePin(motor1Forward_GPIO_Port, motor1Forward_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(motor1Backward_GPIO_Port, motor1Backward_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(motor2Forward_GPIO_Port, motor2Forward_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(motor2Backward_GPIO_Port, motor2Backward_Pin, GPIO_PIN_RESET);

}


void sharpright(uint16_t speed){
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);  // Change motor 2 speed
	  HAL_GPIO_WritePin(motor1Forward_GPIO_Port, motor1Forward_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(motor1Backward_GPIO_Port, motor1Backward_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(motor2Forward_GPIO_Port, motor2Forward_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(motor2Backward_GPIO_Port, motor2Backward_Pin, GPIO_PIN_SET);

}

void reverse(uint16_t speed){
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);  // Change motor 2 speed
	  HAL_GPIO_WritePin(motor1Forward_GPIO_Port, motor1Forward_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(motor1Backward_GPIO_Port, motor1Backward_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(motor2Forward_GPIO_Port, motor2Forward_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(motor2Backward_GPIO_Port, motor2Backward_Pin, GPIO_PIN_SET);
}


// Stop all motors
void stopBot() {
    HAL_GPIO_WritePin(motor2Forward_GPIO_Port, motor1Forward_Pin, GPIO_PIN_RESET);  // Stop motor 1 forward
    HAL_GPIO_WritePin(motor2Backward_GPIO_Port, motor1Backward_Pin, GPIO_PIN_RESET);  // Stop motor 1 backward
    HAL_GPIO_WritePin(motor2Forward_GPIO_Port, motor2Forward_Pin, GPIO_PIN_RESET);  // Stop motor 2 forward
    HAL_GPIO_WritePin(motor2Backward_GPIO_Port, motor2Backward_Pin, GPIO_PIN_RESET);  // Stop motor 2 backward
}

// Activate braking for motors
void Frein() {
    HAL_GPIO_WritePin(motor2Forward_GPIO_Port, motor1Forward_Pin, GPIO_PIN_SET);  // Apply brake on motor 1 forward
    HAL_GPIO_WritePin(motor2Backward_GPIO_Port, motor1Backward_Pin, GPIO_PIN_SET);  // Apply brake on motor 1 backward
    HAL_GPIO_WritePin(motor2Forward_GPIO_Port, motor2Forward_Pin, GPIO_PIN_SET);  // Apply brake on motor 2 forward
    HAL_GPIO_WritePin(motor2Backward_GPIO_Port, motor2Backward_Pin, GPIO_PIN_SET);  // Apply brake on motor 2 backward
}