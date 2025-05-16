// light_sensor_task.h
#ifndef LIGHT_SENSOR_TASK_H
#define LIGHT_SENSOR_TASK_H

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
// Structure for message queue
typedef struct AMessage
{
    uint32_t ulTimeStamp;
    uint32_t uRaw;
    uint32_t uFiltered;
} AMessage;

// Function declarations
void vCreateLightSensorTask(void);


#endif