#ifndef SERVO_MANAGER_H_
#define SERVO_MANAGER_H_

#include "main.h"

// Define the 5 servos based on your pinout
typedef enum {
    SERVO_1 = 0, // PA1, TIM2_CH2
    SERVO_2 = 1, // PA2, TIM2_CH3
    SERVO_3 = 2, // PA3, TIM2_CH4
    SERVO_4 = 3, // PA5, TIM2_CH1
    SERVO_5 = 4  // PA6, TIM3_CH1
} Servo_ID_t;

// Initialize the PWM timers for all servos
void Servo_Init(void);

// Set the angle of a specific servo (0 to 180 degrees)
void Servo_SetAngle(Servo_ID_t servo_id, float angle);

// Set the direct pulse width in microseconds (e.g., 1500 for center)
void Servo_SetPulse(Servo_ID_t servo_id, uint16_t pulse_us);

#endif /* SERVO_MANAGER_H_ */
