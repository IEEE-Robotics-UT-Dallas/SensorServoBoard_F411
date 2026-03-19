#ifndef SENSOR_TASKS_H_
#define SENSOR_TASKS_H_

#include "main.h"

// Task entry points for FreeRTOS
void StartI2C1Task(void *argument);
void StartI2C2Task(void *argument);
void StartI2C3Task(void *argument);
void StartServoTask(void *argument);

#endif /* SENSOR_TASKS_H_ */
