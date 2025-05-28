#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"
/*
 * Queue used to send and receive complete struct AMessage structures.
 */

extern QueueHandle_t xPointerQueue;

extern EventGroupHandle_t xEventGroup;
/*
 * Time stamp global variable.
 */
extern volatile uint32_t g_ui32TimeStamp;
typedef struct AMessage
{
    uint32_t ulTimeStamp;
    uint32_t uRaw;
    uint32_t uFiltered;
} AMessage;
/*
 * The number of items the queue can hold.  This is 4 as the receive task
 * will remove items as they are added, meaning the send task should always find
 * the queue empty.
 */
#define mainQUEUE_LENGTH (4)
#endif // COMMON_H