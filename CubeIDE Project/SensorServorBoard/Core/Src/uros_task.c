#include "uros_task.h"

#ifdef MICRO_ROS_ENABLED

#include "shared_data.h"
#include "servo_control.h"

#include "stm32f4xx_hal.h"
#include "main.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/publisher.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "uros_transport.h"

/* Allocator functions (microros_allocators.c) */
void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

extern UART_HandleTypeDef huart6;

/* Publishers (3 total) */
static rcl_publisher_t mag_pub;        /* /imu/mag — MagneticField with timestamp */
static rcl_publisher_t telemetry_pub;  /* /telemetry — [tof×5, mag×3, light×1] */
static rcl_publisher_t servo_pos_pub;  /* /servo_pos — 5 servo angles (degrees) */

/* Subscriptions */
static rcl_subscription_t servo_cmd_sub;

/* Timers */
static rcl_timer_t sensor_timer;

/* Time sync state */
static volatile bool time_synced = false;

static void fill_timestamp(builtin_interfaces__msg__Time *stamp)
{
    if (time_synced) {
        int64_t ns = rmw_uros_epoch_nanos();
        stamp->sec  = (int32_t)(ns / 1000000000LL);
        stamp->nanosec = (uint32_t)(ns % 1000000000LL);
    } else {
        /* Fallback: FreeRTOS tick (ms since boot) */
        uint32_t ms = HAL_GetTick();
        stamp->sec  = (int32_t)(ms / 1000);
        stamp->nanosec = (ms % 1000) * 1000000U;
    }
}

/* Servo command subscriber callback */
static float servo_angles[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

static void servo_cmd_callback(const void * msgin)
{
    const std_msgs__msg__Float32MultiArray * msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;

    if (msg == NULL || msg->data.size < 5) return;

    for (int i = 0; i < 5; i++)
    {
        servo_angles[i] = msg->data.data[i];
        ServoControl_SetAngle((ServoCtrl_ID_t)i, servo_angles[i]);
    }
}

/* Sensor publishing timer callback (20 Hz) */
static void sensor_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    if (sensor_data_mutex == NULL) return;

    osMutexAcquire(sensor_data_mutex, osWaitForever);

    /* Publish magnetometer (with timestamp) */
    sensor_msgs__msg__MagneticField mag_msg;
    memset(&mag_msg, 0, sizeof(mag_msg));
    fill_timestamp(&mag_msg.header.stamp);
    mag_msg.magnetic_field.x = (double)g_sensor_data.mag_data.x;
    mag_msg.magnetic_field.y = (double)g_sensor_data.mag_data.y;
    mag_msg.magnetic_field.z = (double)g_sensor_data.mag_data.z;
    rcl_publish(&mag_pub, &mag_msg, NULL);

    /* Aggregated telemetry (Float32MultiArray): [tof×5, mag×3, light×1] */
    static float telem_data[TELEMETRY_SIZE];
    shared_data_to_telemetry(&g_sensor_data, telem_data, TELEMETRY_SIZE);

    std_msgs__msg__Float32MultiArray telem_msg;
    memset(&telem_msg, 0, sizeof(telem_msg));
    telem_msg.data.data = telem_data;
    telem_msg.data.size = TELEMETRY_SIZE;
    telem_msg.data.capacity = TELEMETRY_SIZE;
    rcl_publish(&telemetry_pub, &telem_msg, NULL);

    osMutexRelease(sensor_data_mutex);

    /* Publish servo positions (outside mutex — servo_angles is local) */
    std_msgs__msg__Float32MultiArray servo_msg;
    memset(&servo_msg, 0, sizeof(servo_msg));
    servo_msg.data.data = servo_angles;
    servo_msg.data.size = 5;
    servo_msg.data.capacity = 5;
    rcl_publish(&servo_pos_pub, &servo_msg, NULL);
}

void StartuROSTask(void *argument)
{
    (void)argument;

    /* Start servo PWM timers */
    ServoControl_Init();

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

    /* Create magnetometer publisher (timestamped) */
    rclc_publisher_init_default(
        &mag_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        "imu/mag"
    );

    /* Create telemetry publisher [tof×5, mag×3, light×1] */
    rclc_publisher_init_default(
        &telemetry_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "telemetry"
    );

    /* Create servo position feedback publisher */
    rclc_publisher_init_default(
        &servo_pos_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "servo_pos"
    );

    /* Create servo command subscription */
    rclc_subscription_init_default(
        &servo_cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "servo_cmd"
    );

    /* Create sensor publishing timer at 50 Hz */
    rclc_timer_init_default2(
        &sensor_timer,
        &support,
        RCL_MS_TO_NS(20),
        sensor_timer_callback,
        true
    );

    /* Initialize executor: 1 timer + 1 subscription = 2 handles */
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &sensor_timer);

    /* Pre-allocate subscription message buffer */
    static std_msgs__msg__Float32MultiArray servo_cmd_msg;
    static float servo_cmd_data[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    servo_cmd_msg.data.data = servo_cmd_data;
    servo_cmd_msg.data.capacity = 5;
    servo_cmd_msg.data.size = 5;

    rclc_executor_add_subscription(
        &executor,
        &servo_cmd_sub,
        &servo_cmd_msg,
        &servo_cmd_callback,
        ON_NEW_DATA
    );

    /* Spin forever */
    for(;;)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
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
