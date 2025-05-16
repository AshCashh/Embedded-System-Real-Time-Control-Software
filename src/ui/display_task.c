#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "grlib.h"
#include "event_groups.h"
#include "utils/uartstdio.h"
#include "includes/display_task.h"
#include "includes/light_sensor_task.h"
#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "includes/common.h"

extern uint32_t g_ui32SysClock;
extern EventGroupHandle_t xEventGroup;

static void vPlotSensorData(uint32_t *data, int dataSize);

tContext sContext;
/*
 * Queue used to send and receive complete struct AMessage structures.
 */

static void prvDisplayTask(void *pvParameters);

void vCreateDisplayTask(void)
{

    if (xStructQueue == NULL)
    {
        UARTprintf("Error: xStructQueue is not initialized!\n");
        return;
    }
    xTaskCreate(prvDisplayTask,
                "Display Task",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                NULL);
}

static void prvDisplayTask(void *pvParameters)
{
    tRectangle sRect;

    //
    // Initialize the display driver.
    //
    Kentec320x240x16_SSD2119Init(configCPU_CLOCK_HZ);

    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

    //
    // Fill the top 24 rows of the screen with blue to create the banner.
    //

    bool plotRawData = false;

    struct AMessage xRxedStructure;
    uint32_t buffer_data[100] = {0};
    int data_index = 0;
    for (;;)
    {
        // Check for event bits
        EventBits_t uxBits = xEventGroupWaitBits(
            xEventGroup,
            EVENT_HIGH_THRESHOLD | EVENT_LOW_THRESHOLD | EVENT_BTN_TOGGLE,
            pdTRUE,  // Clear bits after reading
            pdFALSE, // Wait for any bit
            0);      // Non-blocking
        if (uxBits == 0)
        {
            // UARTprintf("Warning: No event bits set\n");
        }

        if (uxBits & EVENT_HIGH_THRESHOLD)
        {
            UARTprintf("Warning: High threshold exceeded!\n");
        }

        if (uxBits & EVENT_LOW_THRESHOLD)
        {
            UARTprintf("Warning: Low threshold exceeded!\n");
        }

        if (uxBits & EVENT_BTN_TOGGLE)
        {
            plotRawData = !plotRawData;
            UARTprintf("Toggled plot mode: %s\n", plotRawData ? "Raw Data" : "Filtered Data");
        }

        if (xQueueReceive(xStructQueue, &(xRxedStructure), (TickType_t)10) == pdPASS)
        {
            buffer_data[data_index] = plotRawData ? xRxedStructure.uRaw : xRxedStructure.uFiltered;
            data_index = (data_index + 1) % 100;
            UARTprintf("%d\n", plotRawData ? xRxedStructure.uRaw : xRxedStructure.uFiltered);
            /* xRxedStructure now contains a copy of xMessage. */
            // UARTprintf("Receiving Task: raw %d, filtered %d\n", xRxedStructure.uRaw, xRxedStructure.uFiltered);

            vPlotSensorData(buffer_data, data_index);
        }
        else
        {
            // UARTprintf("Error: No data received from the queue\n");
        }
    }
}

static void vPlotSensorData(uint32_t *data, int dataSize)
{
    uint32_t xStart = 20;  // Starting X position for the plot
    uint32_t yStart = 220; // Starting Y position (bottom of the graph)
    uint32_t xStep = 3;    // Distance between points on the X-axis
    uint32_t yScale = 2;   // Scale factor for Y-axis

    // Draw the axes
    GrContextForegroundSet(&sContext, ClrWhite);
    GrLineDraw(&sContext, 20, 220, 300, 220); // X-axis
    GrLineDraw(&sContext, 20, 27, 20, 220);   // Y-axis
    // Add labels for the axes
    GrContextFontSet(&sContext, g_psFontFixed6x8);
    GrStringDraw(&sContext, "Time", -1, 150, 230, false); // X-axis label
    GrStringDraw(&sContext, "Lux", -1, 4, 9, false);      // Y-axis label

    // Plot the data points
    GrContextForegroundSet(&sContext, ClrRed);
    for (uint32_t i = 0; i < ((dataSize - 1) < 0 ? 0 : dataSize - 1); i++)
    {
        uint32_t x1 = xStart + (i * xStep);
        uint32_t y1 = yStart - (data[i] / yScale);
        uint32_t x2 = xStart + ((i + 1) * xStep);
        uint32_t y2 = yStart - (data[i + 1] / yScale);

        GrLineDraw(&sContext, x1, y1, x2, y2);
    }

    // Clear the plotting area
    tRectangle sRect = {21, 0, 350, 220};
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);
}
