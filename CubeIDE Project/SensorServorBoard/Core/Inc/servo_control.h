#ifndef SERVO_CONTROL_H_
#define SERVO_CONTROL_H_

#include "main.h"

#define SERVO_COUNT 3

typedef enum {
    SERVO_CTRL_1 = 0, // PA1, TIM2_CH2
    SERVO_CTRL_2 = 1, // PA2, TIM2_CH3
    SERVO_CTRL_3 = 2  // PA3, TIM2_CH4
} ServoCtrl_ID_t;

void ServoControl_Init(void);
void ServoControl_SetAngle(ServoCtrl_ID_t servo_id, float angle);
void ServoControl_SetPulse(ServoCtrl_ID_t servo_id, uint16_t pulse_us);

/* Get the per-servo zero offset (degrees added to physical angle) */
float ServoControl_GetOffset(ServoCtrl_ID_t servo_id);

/* Set the per-servo zero offset at runtime */
void ServoControl_SetOffset(ServoCtrl_ID_t servo_id, float offset_deg);

#endif /* SERVO_CONTROL_H_ */
