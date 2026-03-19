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
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "uros_transport.h"

/* Allocator functions (microros_allocators.c) */
void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

extern UART_HandleTypeDef huart1;

/* Publishers */
static rcl_publisher_t tof_pub[5];
static rcl_publisher_t mag_pub;
static rcl_publisher_t light_pub;
static rcl_publisher_t telemetry_pub;

/* Subscriptions */
static rcl_subscription_t servo_cmd_sub;

/* Timers */
static rcl_timer_t sensor_timer;

/* Servo command subscriber callback */
static void servo_cmd_callback(const void * msgin)
{
    const std_msgs__msg__Float32MultiArray * msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;

    if (msg == NULL || msg->data.size < 5) return;

    for (int i = 0; i < 5; i++)
    {
        ServoControl_SetAngle((ServoCtrl_ID_t)i, msg->data.data[i]);
    }
}

/* Sensor publishing timer callback (20 Hz) */
static void sensor_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    if (sensor_data_mutex == NULL) return;

    osMutexAcquire(sensor_data_mutex, osWaitForever);

    /* Publish 5 ToF range messages */
    for (int i = 0; i < 5; i++)
    {
        sensor_msgs__msg__Range tof_msg;
        memset(&tof_msg, 0, sizeof(tof_msg));
        tof_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
        tof_msg.field_of_view = 0.471f;  /* ~27 deg for VL53L0X */
        tof_msg.min_range = 0.03f;
        tof_msg.max_range = 2.0f;
        tof_msg.range = (float)g_sensor_data.tof_distances[i] / 1000.0f;
        rcl_publish(&tof_pub[i], &tof_msg, NULL);
    }

    /* Publish magnetometer */
    sensor_msgs__msg__MagneticField mag_msg;
    memset(&mag_msg, 0, sizeof(mag_msg));
    mag_msg.magnetic_field.x = (double)g_sensor_data.mag_data.x;
    mag_msg.magnetic_field.y = (double)g_sensor_data.mag_data.y;
    mag_msg.magnetic_field.z = (double)g_sensor_data.mag_data.z;
    rcl_publish(&mag_pub, &mag_msg, NULL);

    /* Publish light level */
    std_msgs__msg__Float32 light_msg;
    memset(&light_msg, 0, sizeof(light_msg));
    light_msg.data = (float)g_sensor_data.light_lux;
    rcl_publish(&light_pub, &light_msg, NULL);

    /* Aggregated telemetry (Float32MultiArray) */
    static float telem_data[TELEMETRY_SIZE];
    shared_data_to_telemetry(&g_sensor_data, telem_data, TELEMETRY_SIZE);

    std_msgs__msg__Float32MultiArray telem_msg;
    memset(&telem_msg, 0, sizeof(telem_msg));
    telem_msg.data.data = telem_data;
    telem_msg.data.size = TELEMETRY_SIZE;
    telem_msg.data.capacity = TELEMETRY_SIZE;
    rcl_publish(&telemetry_pub, &telem_msg, NULL);

    osMutexRelease(sensor_data_mutex);
}

void StartuROSTask(void *argument)
{
    (void)argument;

    /* Register custom UART DMA transport */
    rmw_uros_set_custom_transport(
        true,
        (void *) &huart1,
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

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "sensor_servo_board", "", &support);

    /* Create ToF range publishers (tof_0 .. tof_4) */
    for (int i = 0; i < 5; i++)
    {
        char topic_name[16];
        snprintf(topic_name, sizeof(topic_name), "tof_%d", i);
        rclc_publisher_init_default(
            &tof_pub[i],
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
            topic_name
        );
    }

    /* Create magnetometer publisher */
    rclc_publisher_init_default(
        &mag_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        "imu/mag"
    );

    /* Create light sensor publisher */
    rclc_publisher_init_default(
        &light_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "light"
    );

    /* Create aggregated telemetry publisher */
    rclc_publisher_init_default(
        &telemetry_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "telemetry"
    );

    /* Create servo command subscription */
    rclc_subscription_init_default(
        &servo_cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "servo_cmd"
    );

    /* Create sensor publishing timer at 20 Hz */
    rclc_timer_init_default2(
        &sensor_timer,
        &support,
        RCL_MS_TO_NS(50),
        sensor_timer_callback,
        true
    );

    /* Initialize executor: 1 timer + 1 subscription = 2 handles */
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &sensor_timer);

    /* Pre-allocate subscription message buffer */
    static std_msgs__msg__Float32MultiArray servo_cmd_msg;
    static float servo_cmd_data[5] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f};
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
