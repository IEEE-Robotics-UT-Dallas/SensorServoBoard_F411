#include "servo_control.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/*
 * Per-servo zero offset (degrees). Physical 0° = offset in raw servo degrees.
 * Calibrated values: command = physical_angle + offset
 * E.g., servo 0 offset=135 means SetAngle(0) sends 135° raw pulse.
 */
static float servo_offset[SERVO_COUNT] = {
    135.0f,  /* SERVO_CTRL_1 (PA1) — calibrated */
     87.0f,  /* SERVO_CTRL_2 (PA2) — calibrated */
      0.0f,  /* SERVO_CTRL_3 (PA3) — uncalibrated */
      0.0f,  /* SERVO_CTRL_4 (PA5) — uncalibrated */
      0.0f   /* SERVO_CTRL_5 (PA6) — uncalibrated */
};

float ServoControl_GetOffset(ServoCtrl_ID_t servo_id)
{
    if (servo_id >= SERVO_COUNT) return 0.0f;
    return servo_offset[servo_id];
}

void ServoControl_SetOffset(ServoCtrl_ID_t servo_id, float offset_deg)
{
    if (servo_id >= SERVO_COUNT) return;
    servo_offset[servo_id] = offset_deg;
}

void ServoControl_Init(void) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // SERVO_CTRL_1 (PA1)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // SERVO_CTRL_2 (PA2)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // SERVO_CTRL_3 (PA3)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // SERVO_CTRL_4 (PA5)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // SERVO_CTRL_5 (PA6)

    /* Boot connected servos to physical 0° (flush) */
    ServoControl_SetAngle(SERVO_CTRL_1, 0.0f);
    ServoControl_SetAngle(SERVO_CTRL_2, 0.0f);
}

void ServoControl_SetPulse(ServoCtrl_ID_t servo_id, uint16_t pulse_us) {
    switch(servo_id) {
        case SERVO_CTRL_1: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_us); break;
        case SERVO_CTRL_2: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse_us); break;
        case SERVO_CTRL_3: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse_us); break;
        case SERVO_CTRL_4: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us); break;
        case SERVO_CTRL_5: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us); break;
    }
}

void ServoControl_SetAngle(ServoCtrl_ID_t servo_id, float angle) {
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    /* Apply calibration offset: raw = physical + offset */
    float raw = angle + servo_offset[servo_id];
    if (raw < 0.0f) raw = 0.0f;
    if (raw > 180.0f) raw = 180.0f;

    uint16_t pulse = (uint16_t)(500.0f + (raw / 180.0f) * 2000.0f);
    ServoControl_SetPulse(servo_id, pulse);
}
