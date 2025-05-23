#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "event_groups.h"

// Function prototypes
void vCreateDisplayTask(void);
void prvConfigureHWTimer(void);
void xTimerHandler(void);
#endif // DISPLAY_TASK_H