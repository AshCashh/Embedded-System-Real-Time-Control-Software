#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "event_groups.h"

// Function prototypes
void vCreateDisplayTask(void);
void xTimerHandler(void);
#endif // DISPLAY_TASK_H

extern uint32_t accel_threshold;
extern uint32_t current_threshold;