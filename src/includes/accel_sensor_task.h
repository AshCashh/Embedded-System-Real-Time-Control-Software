// light_sensor_task.h
#ifndef ACCEL_SENSOR_TASK_H
#define ACCEL_SENSOR_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
// Event bits
#define EVENT_HIGH_THRESHOLD (1 << 0)
#define EVENT_LOW_THRESHOLD (1 << 1)
#define EVENT_BTN_TOGGLE (1 << 2)

// Threshold values
#define HIGH_THRESHOLD 1000
#define LOW_THRESHOLD 50

#define FILTER_SIZE 10

// Acceleration queue
extern QueueHandle_t xAccelQueue;

typedef struct {
    uint32_t ulTimeStamp;
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} BMI_s;


// Function declarations
void vCreateLightSensorTask(void);
void vCreateAccelTask(void);


#endif