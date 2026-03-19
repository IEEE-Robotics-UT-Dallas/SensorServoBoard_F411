#ifndef SERVO_CONTROL_H_
#define SERVO_CONTROL_H_

#include "main.h"

// Define the 5 servos based on the pinout
typedef enum {
    SERVO_CTRL_1 = 0, // PA1, TIM2_CH2
    SERVO_CTRL_2 = 1, // PA2, TIM2_CH3
    SERVO_CTRL_3 = 2, // PA3, TIM2_CH4
    SERVO_CTRL_4 = 3, // PA5, TIM2_CH1
    SERVO_CTRL_5 = 4  // PA6, TIM3_CH1
} ServoCtrl_ID_t;

// Initialize the PWM timers for all servos
void ServoControl_Init(void);

// Set the angle of a specific servo (0 to 180 degrees)
void ServoControl_SetAngle(ServoCtrl_ID_t servo_id, float angle);

// Set the direct pulse width in microseconds (e.g., 1500 for center)
void ServoControl_SetPulse(ServoCtrl_ID_t servo_id, uint16_t pulse_us);

#endif /* SERVO_CONTROL_H_ */
