
#ifndef QTR_STM_H
#define QTR_STM_H

#include <string.h>
#include <stdio.h>


// Define constants for GPIO modes
#define OUTPUT GPIO_MODE_OUTPUT_PP
#define INPUT GPIO_MODE_INPUT

// Define the number of sensors
#define sensorCount 16

// Define the maximum sensor reading value in microseconds
#define maxValue 32000

// Microseconds per system tick constant  --depends on HCLK
#define micros_per_sysTick 0.0055555555556

// Timer handler declarations
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

// Start base timer and PWM channels
/* USER CODE BEGIN 2 */
HAL_TIM_Base_Start(&htim1);
EnableTiming();
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

// UART handler declaration
UART_HandleTypeDef huart2;

// Debug configuration for ITM
static int Debug_ITMDebug = 0;

// Define memory-mapped debug registers for timing
volatile unsigned int *DWT_CYCCNT   = (volatile unsigned int *)0xE0001004;
volatile unsigned int *DWT_CONTROL  = (volatile unsigned int *)0xE0001000;
volatile unsigned int *DWT_LAR      = (volatile unsigned int *)0xE0001FB0;
volatile unsigned int *SCB_DHCSR    = (volatile unsigned int *)0xE000EDF0;
volatile unsigned int *SCB_DEMCR    = (volatile unsigned int *)0xE000EDFC;
volatile unsigned int *ITM_TER      = (volatile unsigned int *)0xE0000E00;
volatile unsigned int *ITM_TCR      = (volatile unsigned int *)0xE0000E80;

// Define ports corresponding to each sensor pin
GPIO_TypeDef* sensorPorts[] = {
    GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOC, GPIOC, GPIOC,
    GPIOC, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB
};

// Define pin numbers for each sensor
uint16_t sensorPins[] = {void delay_us(uint16_t us);
    GPIO_PIN_12, GPIO_PIN_11, GPIO_PIN_10, GPIO_PIN_9,void EnableTiming(void);
    GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_8, GPIO_PIN_7,
    GPIO_PIN_6, GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13,void pinMode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t Mode);
    GPIO_PIN_12, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_10
};

// Variable to store sensor status as a bitfieldvoid readDigit(uint16_t *sensorValues, CalibrationData* calibration);
extern uint16_t sensorBits = 0;
// Arrays to store sensor states and thresholds
uint8_t ir[sensorCount];

uint16_t sensorValues[sensorCount];
uint16_t threshhold[sensorCount];
uint16_t readSensorBits(void);

// Define white and black statesvoid Error_Handler(void);
extern uint8_t B = 1;
extern uint8_t W = 0;

// Structure to hold calibration data for sensors
typedef struct {
    uint16_t maximum[sensorCount];
    uint16_t minimum[sensorCount];
} CalibrationData;

// Initialize calibration data
CalibrationData calibration = {};

// Delay execution for a specified number of microseconds
void delay_us(uint16_t us);

// Enable precise timing by configuring the necessary hardware counters
void EnableTiming(void);

// Configure the mode (INPUT or OUTPUT) for a specific GPIO pin on a specified port
void pinMode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t Mode);

// Read values from sensors and store them in an array, used for real-time sensing
void readSensors(uint16_t *sensorValues);

// Interpret sensor readings as digital values (0 or 1) based on threshold, using calibration data
void readDigit(uint16_t *sensorValues, CalibrationData* calibration);

// Calibrate sensors by determining minimum and maximum values over multiple readings
void calibrate(CalibrationData* calibration);

// Calculate threshold values for each sensor based on calibration data
void getThreshhold(CalibrationData* calibration);

// Read and return the current sensor readings as a bit-packed integer
uint16_t readSensorBits(void);

// Handle critical errors by disabling interrupts and entering an infinite loop
void Error_Handler(void);



#endif
