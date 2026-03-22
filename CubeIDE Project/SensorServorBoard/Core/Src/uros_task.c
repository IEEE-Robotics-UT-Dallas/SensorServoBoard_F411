#include "uros_task.h"

#ifdef MICRO_ROS_ENABLED

#include "shared_data.h"
#include "double_buffer.h"
#include "servo_control.h"
#include "flash_config.h"

#include "stm32f4xx_hal.h"
#include "main.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/publisher.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "uros_transport.h"

/* Forward-declare FreeRTOS-backed micro-ROS allocators (microros_allocators.c) */
extern void * microros_allocate(size_t size, void * state);
extern void   microros_deallocate(void * pointer, void * state);
extern void * microros_reallocate(void * pointer, size_t size, void * state);
extern void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

extern UART_HandleTypeDef huart6;

/* Board Configuration & Flash Persistence */
static BoardConfig_t g_config;
static bool g_param_modified = false;
static uint32_t g_last_param_update_ms = 0;
#define FLASH_WRITE_SETTLE_MS 2000

/* Time sync state */
static bool time_synced = false;

/* Current servo commanded angles (published and updated by subscriber) */
static float servo_angles[3] = {90.0f, 90.0f, 90.0f};

/* Publishers */
static rcl_publisher_t mag_pub;
static rcl_publisher_t telemetry_pub;
static rcl_publisher_t servo_pos_pub;

/* Parameter Server
static rclc_parameter_server_t param_server;
*/

/* Subscriptions */
static rcl_subscription_t servo_cmd_sub;
static std_msgs__msg__Float32MultiArray servo_cmd_msg;
static float servo_cmd_data[3];

/* Timers */
static rcl_timer_t sensor_timer;

/* Fill a ROS timestamp from synced epoch time, or fallback to HAL_GetTick */
static void fill_timestamp(builtin_interfaces__msg__Time * stamp)
{
    if (time_synced) {
        int64_t nanos = rmw_uros_epoch_nanos();
        stamp->sec = (int32_t)(nanos / 1000000000LL);
        stamp->nanosec = (uint32_t)(nanos % 1000000000LL);
    } else {
        uint32_t ms = HAL_GetTick();
        stamp->sec = (int32_t)(ms / 1000);
        stamp->nanosec = (uint32_t)((ms % 1000) * 1000000UL);
    }
}

static void update_servo_offsets_from_config(void) {
    for (int i = 0; i < 3; i++) {
        ServoControl_SetOffset((ServoCtrl_ID_t)i, g_config.servo_offsets[i]);
    }
}

/* Parameter modification callback */
bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context) {
    (void)context;
    (void)old_param;
    if (new_param == NULL) return false;

    if (strcmp(new_param->name.data, "servo_offset_1") == 0) {
        g_config.servo_offsets[0] = (float)new_param->value.double_value;
    } else if (strcmp(new_param->name.data, "servo_offset_2") == 0) {
        g_config.servo_offsets[1] = (float)new_param->value.double_value;
    } else if (strcmp(new_param->name.data, "servo_offset_3") == 0) {
        g_config.servo_offsets[2] = (float)new_param->value.double_value;
    } else if (strcmp(new_param->name.data, "servo_offset_4") == 0) {
        g_config.servo_offsets[3] = (float)new_param->value.double_value;
    } else if (strcmp(new_param->name.data, "servo_offset_5") == 0) {
        g_config.servo_offsets[4] = (float)new_param->value.double_value;
    } else {
        return false;
    }

    update_servo_offsets_from_config();
    g_param_modified = true;
    g_last_param_update_ms = HAL_GetTick();
    return true;
}

/* Servo command subscriber callback */
static void servo_cmd_callback(const void * msgin)
{
    const std_msgs__msg__Float32MultiArray * msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;

    if (msg == NULL || msg->data.data == NULL) return;

    size_t count = msg->data.size;
    if (count > 3) count = 3;

    for (size_t i = 0; i < count; i++) {
        servo_angles[i] = msg->data.data[i];
        ServoControl_SetAngle((ServoCtrl_ID_t)i, servo_angles[i]);
    }
}

/* Sensor publishing timer callback (20 Hz) */
static void sensor_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    /* Handle Flash Persistence Settle Logic */
    if (g_param_modified && (HAL_GetTick() - g_last_param_update_ms > FLASH_WRITE_SETTLE_MS)) {
        Flash_SaveConfig(&g_config);
        g_param_modified = false;
    }

    const sensor_snapshot_t *snap = DoubleBuffer_GetReadBuffer(&g_sensor_db);

    /* Use static message structs to avoid stack overflow — MagneticField
     * alone is ~120+ bytes (72B covariance + header + Vector3). */
    static sensor_msgs__msg__MagneticField mag_msg;
    memset(&mag_msg, 0, sizeof(mag_msg));
    fill_timestamp(&mag_msg.header.stamp);
    mag_msg.magnetic_field.x = (double)snap->data.mag_data.x;
    mag_msg.magnetic_field.y = (double)snap->data.mag_data.y;
    mag_msg.magnetic_field.z = (double)snap->data.mag_data.z;
    rcl_ret_t rc = rcl_publish(&mag_pub, &mag_msg, NULL);
    (void)rc;

    /* Aggregated telemetry (Float32MultiArray): [tof×5, mag×3, light×1] */
    static float telem_data[TELEMETRY_SIZE];
    shared_data_to_telemetry(&snap->data, telem_data, TELEMETRY_SIZE);

    static std_msgs__msg__Float32MultiArray telem_msg;
    memset(&telem_msg, 0, sizeof(telem_msg));
    telem_msg.data.data = telem_data;
    telem_msg.data.size = TELEMETRY_SIZE;
    telem_msg.data.capacity = TELEMETRY_SIZE;
    rc = rcl_publish(&telemetry_pub, &telem_msg, NULL);
    (void)rc;

    /* Publish servo positions */
    static std_msgs__msg__Float32MultiArray servo_msg;
    memset(&servo_msg, 0, sizeof(servo_msg));
    servo_msg.data.data = servo_angles;
    servo_msg.data.size = 3;
    servo_msg.data.capacity = 3;
    rc = rcl_publish(&servo_pos_pub, &servo_msg, NULL);
    (void)rc;
}

void StartuROSTask(void *argument)
{
    (void)argument;

    /* Start servo PWM timers */
    ServoControl_Init();

    /* Initialize configuration from Flash */
    if (Flash_LoadConfig(&g_config) != HAL_OK) {
        Flash_GetDefaultConfig(&g_config);
        Flash_SaveConfig(&g_config);
    }
    update_servo_offsets_from_config();

    /* Register custom UART DMA transport */
    rmw_uros_set_custom_transport(
        true,
        (void *) &huart6,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read
    );

    /* Configure FreeRTOS-backed allocator for micro-ROS */
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator))
    {
        Error_Handler();
    }

    /* Initialize RCL support, node */
    rclc_executor_t executor;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;
    rcl_ret_t rc;

    allocator = rcl_get_default_allocator();

    rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK)
    {
        /* Transport or session failure — retry after delay */
        for (;;) { osDelay(1000); }
    }

    rc = rclc_node_init_default(&node, "sensor_servo_board", "", &support);
    if (rc != RCL_RET_OK)
    {
        for (;;) { osDelay(1000); }
    }

    /* ---- Parameter server temporarily disabled ----
     * The static library was built with RMW_UXRCE_MAX_SERVICES=1,
     * but the parameter server needs ~5 services.  Re-enable after
     * rebuilding libmicroros.a with MAX_SERVICES>=6.
     */
#if 0  /* PARAM_SERVER — disabled until library rebuild */
    /* Initialize parameter server */
    rc = rclc_parameter_server_init_default(&param_server, &node);
    if (rc != RCL_RET_OK) { Error_Handler(); }

    /* Register parameters */
    rclc_add_parameter(&param_server, "servo_offset_1", RCLC_PARAMETER_DOUBLE);
    rclc_parameter_set_double(&param_server, "servo_offset_1", g_config.servo_offsets[0]);

    rclc_add_parameter(&param_server, "servo_offset_2", RCLC_PARAMETER_DOUBLE);
    rclc_parameter_set_double(&param_server, "servo_offset_2", g_config.servo_offsets[1]);

    rclc_add_parameter(&param_server, "servo_offset_3", RCLC_PARAMETER_DOUBLE);
    rclc_parameter_set_double(&param_server, "servo_offset_3", g_config.servo_offsets[2]);

    rclc_add_parameter(&param_server, "servo_offset_4", RCLC_PARAMETER_DOUBLE);
    rclc_parameter_set_double(&param_server, "servo_offset_4", g_config.servo_offsets[3]);

    rclc_add_parameter(&param_server, "servo_offset_5", RCLC_PARAMETER_DOUBLE);
    rclc_parameter_set_double(&param_server, "servo_offset_5", g_config.servo_offsets[4]);
#endif

    /* Synchronize clock with agent (NTP-like, 3 attempts) */
    for (int i = 0; i < 3; i++)
    {
        if (rmw_uros_sync_session(500) == RMW_RET_OK)
        {
            time_synced = true;
            break;
        }
        osDelay(200);
    }

    /* Initialize publishers */
    rc = rclc_publisher_init_default(&mag_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "imu/mag");
    if (rc != RCL_RET_OK) { Error_Handler(); }

    rc = rclc_publisher_init_default(&telemetry_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "telemetry");
    if (rc != RCL_RET_OK) { Error_Handler(); }

    rc = rclc_publisher_init_default(&servo_pos_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "servo_positions");
    if (rc != RCL_RET_OK) { Error_Handler(); }

    /* Initialize servo command subscription */
    servo_cmd_msg.data.data = servo_cmd_data;
    servo_cmd_msg.data.size = 0;
    servo_cmd_msg.data.capacity = 3;

    rc = rclc_subscription_init_default(&servo_cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "servo_cmd");
    if (rc != RCL_RET_OK) { Error_Handler(); }

    /* Initialize 20 Hz sensor publishing timer (50ms period) */
    rc = rclc_timer_init_default2(&sensor_timer, &support,
        RCL_MS_TO_NS(50), sensor_timer_callback, true);
    if (rc != RCL_RET_OK) { Error_Handler(); }

    /* Initialize executor: 1 timer + 1 subscription (no param server for now) */
    const size_t num_handles = 1 + 1;
    rclc_executor_init(&executor, &support.context, num_handles, &allocator);
    rclc_executor_add_timer(&executor, &sensor_timer);

    /* Add servo command subscription to executor */
    rclc_executor_add_subscription(&executor, &servo_cmd_sub,
        &servo_cmd_msg, &servo_cmd_callback, ON_NEW_DATA);

#if 0  /* PARAM_SERVER — disabled until library rebuild */
    rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);
#endif

    /* Spin forever (blink PC13 rapidly to indicate successful init + spin) */
    for(;;)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        
        static int blink_counter = 0;
        if (blink_counter++ > 10) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            blink_counter = 0;
        }

        osDelay(10);
    }
}

#else /* MICRO_ROS_ENABLED not defined */

#include "servo_control.h"
#include "cmsis_os.h"

#ifdef HW_TEST
#include "hw_test.h"
#endif

/* Stub: when micro-ROS is not available, run HW test or idle */
void StartuROSTask(void *argument)
{
    (void)argument;

    ServoControl_Init();

#ifdef HW_TEST
    HW_Test_Run();
#endif

    for(;;)
    {
        osDelay(1000);
    }
}

#endif /* MICRO_ROS_ENABLED */
