
// Function to configure GPIO pin modes
void pinMode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t Mode) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = Mode;

    // Set pull-up/pull-down settings based on mode
    switch (Mode) {
        case OUTPUT:
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            break;
        case INPUT:
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            break;
    }

    // Initialize GPIO pin with specified settings
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// Delay function to wait for specified microseconds
void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

// Enable timing by configuring debug registers
void EnableTiming(void) {
    if ((*SCB_DHCSR & 1) && (*ITM_TER & 1))  // Enabled?
        Debug_ITMDebug = 1;

    *SCB_DEMCR |= 0x01000000;
    *DWT_LAR = 0xC5ACCE55; // enable access
    *DWT_CYCCNT = 0; // reset the counter
    *DWT_CONTROL |= 1; // enable the counter
}

// Function to read sensor bitfield (all sensors)
uint16_t readSensorBits() {
    sensorBits = 0;
    for (int i = 16; i > 0; i--) {
        sensorBits |= (ir[i - 1] == 1) << (i - 1);
    }
    return sensorBits;
}

// Function to read all sensor values and store in array
void readSensors(uint16_t *sensorValues) {
    for (uint8_t i = 0; i < sensorCount; i++) {
        sensorValues[i] = maxValue;

        // Set sensor pin to output and pull it high
        pinMode(sensorPorts[i], sensorPins[i], OUTPUT);
        HAL_GPIO_WritePin(sensorPorts[i], sensorPins[i], GPIO_PIN_SET);
    }

    // Delay to charge the sensors
    delay_us(15);

    // Disable interrupts for precise timing
    __disable_irq();
    EnableTiming();
    uint32_t startTime = *DWT_CYCCNT;
    uint16_t time = 0;

    // Set all sensor pins to input mode
    for (uint8_t i = 0; i < sensorCount; i++) {
        pinMode(sensorPorts[i], sensorPins[i], INPUT);
    }

    __enable_irq();

    // Wait until all sensor values are read or max time is reached
    while (time < maxValue) {
        __disable_irq();
        time = (*DWT_CYCCNT - startTime) * micros_per_sysTick;

        for (uint8_t i = 0; i < sensorCount; i++) {
            if ((HAL_GPIO_ReadPin(sensorPorts[i], sensorPins[i]) == GPIO_PIN_RESET) && (time < sensorValues[i])) {
                sensorValues[i] = time;  // Record time the pin went low
            }
        }

        __enable_irq();
    }
}

// Function to read digital representation of sensor values
void readDigit(uint16_t *sensorValues, CalibrationData* calibration) {
    readSensors(sensorValues);
    for (int i = 0; i < sensorCount; i++) {
        ir[i] = (sensorValues[i] < threshhold[i]) ? W : B;
    }
}

// Calculate thresholds for each sensor based on calibration data
void getThreshhold(CalibrationData* calibration) {
    for (int i = 0; i < sensorCount; i++) {
        threshhold[i] = (calibration->minimum[i] + calibration->maximum[i]) / 2;
    }
}

// Function to calibrate sensors by reading minimum and maximum values
void calibrate(CalibrationData* calibration) {
    for (uint8_t i = 0; i < sensorCount; i++) {
        calibration->maximum[i] = 0;
        calibration->minimum[i] = maxValue;
    }

    // Perform multiple readings for calibration
    for (uint16_t j = 0; j < Calib_loop_times; j++) {
        readSensors(sensorValues);

        for (uint8_t i = 0; i < sensorCount; i++) {
            if ((j == 0) || (sensorValues[i] > calibration->maximum[i])) {
                calibration->maximum[i] = sensorValues[i];
            }
            if ((j == 0) || (sensorValues[i] < calibration->minimum[i])) {
                calibration->minimum[i] = sensorValues[i];
            }
        }
    }
}

// Error handler in case of system failure
void Error_Handler(void) {
    __disable_irq();
    while (1) {
        // Error loop - system halt
    }
}
