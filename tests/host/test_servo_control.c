/*
 * test_servo_control.c — Tests for ServoControl_SetAngle / SetPulse.
 * Uses hal_stubs tracking functions to verify __HAL_TIM_SET_COMPARE calls.
 */
#include "unity.h"
#include "servo_control.h"

/* Tracking accessors from hal_stubs.c */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef* hal_stub_get_last_tim_htim(void);
extern uint32_t           hal_stub_get_last_tim_channel(void);
extern uint32_t           hal_stub_get_last_tim_compare(void);
extern int                hal_stub_get_tim_call_count(void);
extern void               hal_stub_reset_tim_tracking(void);

void setUp(void) {
    hal_stub_reset_tim_tracking();
}

void tearDown(void) { }

/* ---- SetAngle: 0° → pulse 500 ---- */
void test_set_angle_0_gives_pulse_500(void) {
    ServoControl_SetAngle(SERVO_CTRL_1, 0.0f);
    TEST_ASSERT_EQUAL_UINT32(500, hal_stub_get_last_tim_compare());
}

/* ---- SetAngle: 90° → pulse 1500 ---- */
void test_set_angle_90_gives_pulse_1500(void) {
    ServoControl_SetAngle(SERVO_CTRL_1, 90.0f);
    TEST_ASSERT_EQUAL_UINT32(1500, hal_stub_get_last_tim_compare());
}

/* ---- SetAngle: 180° → pulse 2500 ---- */
void test_set_angle_180_gives_pulse_2500(void) {
    ServoControl_SetAngle(SERVO_CTRL_1, 180.0f);
    TEST_ASSERT_EQUAL_UINT32(2500, hal_stub_get_last_tim_compare());
}

/* ---- SetAngle: negative angle → clamp to 0° → pulse 500 ---- */
void test_set_angle_negative_clamps_to_0(void) {
    ServoControl_SetAngle(SERVO_CTRL_1, -45.0f);
    TEST_ASSERT_EQUAL_UINT32(500, hal_stub_get_last_tim_compare());
}

/* ---- SetAngle: > 180° → clamp to 180° → pulse 2500 ---- */
void test_set_angle_over_180_clamps(void) {
    ServoControl_SetAngle(SERVO_CTRL_1, 270.0f);
    TEST_ASSERT_EQUAL_UINT32(2500, hal_stub_get_last_tim_compare());
}

/* ---- Servo 1: TIM2_CH2 ---- */
void test_servo1_uses_tim2_ch2(void) {
    ServoControl_SetPulse(SERVO_CTRL_1, 1500);
    TEST_ASSERT_EQUAL_PTR(&htim2, hal_stub_get_last_tim_htim());
    TEST_ASSERT_EQUAL_UINT32(TIM_CHANNEL_2, hal_stub_get_last_tim_channel());
}

/* ---- Servo 2: TIM2_CH3 ---- */
void test_servo2_uses_tim2_ch3(void) {
    ServoControl_SetPulse(SERVO_CTRL_2, 1500);
    TEST_ASSERT_EQUAL_PTR(&htim2, hal_stub_get_last_tim_htim());
    TEST_ASSERT_EQUAL_UINT32(TIM_CHANNEL_3, hal_stub_get_last_tim_channel());
}

/* ---- Servo 3: TIM2_CH4 ---- */
void test_servo3_uses_tim2_ch4(void) {
    ServoControl_SetPulse(SERVO_CTRL_3, 1500);
    TEST_ASSERT_EQUAL_PTR(&htim2, hal_stub_get_last_tim_htim());
    TEST_ASSERT_EQUAL_UINT32(TIM_CHANNEL_4, hal_stub_get_last_tim_channel());
}

/* ---- Servo 4: TIM2_CH1 ---- */
void test_servo4_uses_tim2_ch1(void) {
    ServoControl_SetPulse(SERVO_CTRL_4, 1500);
    TEST_ASSERT_EQUAL_PTR(&htim2, hal_stub_get_last_tim_htim());
    TEST_ASSERT_EQUAL_UINT32(TIM_CHANNEL_1, hal_stub_get_last_tim_channel());
}

/* ---- Servo 5: TIM3_CH1 ---- */
void test_servo5_uses_tim3_ch1(void) {
    ServoControl_SetPulse(SERVO_CTRL_5, 1500);
    TEST_ASSERT_EQUAL_PTR(&htim3, hal_stub_get_last_tim_htim());
    TEST_ASSERT_EQUAL_UINT32(TIM_CHANNEL_1, hal_stub_get_last_tim_channel());
}

/* ---- SetPulse: direct pulse value written ---- */
void test_set_pulse_writes_value(void) {
    ServoControl_SetPulse(SERVO_CTRL_1, 1234);
    TEST_ASSERT_EQUAL_UINT32(1234, hal_stub_get_last_tim_compare());
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_set_angle_0_gives_pulse_500);
    RUN_TEST(test_set_angle_90_gives_pulse_1500);
    RUN_TEST(test_set_angle_180_gives_pulse_2500);
    RUN_TEST(test_set_angle_negative_clamps_to_0);
    RUN_TEST(test_set_angle_over_180_clamps);
    RUN_TEST(test_servo1_uses_tim2_ch2);
    RUN_TEST(test_servo2_uses_tim2_ch3);
    RUN_TEST(test_servo3_uses_tim2_ch4);
    RUN_TEST(test_servo4_uses_tim2_ch1);
    RUN_TEST(test_servo5_uses_tim3_ch1);
    RUN_TEST(test_set_pulse_writes_value);
    return UNITY_END();
}
