/*
 * led_task
 *
 * Copyright (C) 2022 Texas Instruments Incorporated
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/******************************************************************************
 *
 * vLEDTask turns on the initial LED, configurations the buttons, and creates
 * prvProcessSwitchInputTask which handles processing the ISR result to adjust
 * the LED per the button inputs.
 *
 * prvProcessSwitchInputTask uses semaphore take to wait until it receives a
 * semaphore from the button ISR.  Once it does, then it processes the button
 * pressed.  Each time the SW1 or SW2 button is pressed, the LED index is
 * updated based on which button has been pressed and the corresponding LED
 * lights up.
 *
 * When either user switch SW1 or SW2 on the EK-TM4C1294XL is pressed, an
 * interrupt is generated and the switch pressed is logged in the global
 * variable g_pui32ButtonPressed.  Then the binary semaphore is given to
 * prvProcessSwitchInputTask before yielding to it.  This is an example of
 * using binary semaphores to defer ISR processing to a task.
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "driverlib/timer.h"
#include "drivers/i2cOptDriver.h"
#include "driverlib/i2c.h"
#include "drivers/opt3001.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/flash.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "grlib.h"
#include "widget.h"
#include "canvas.h"
#include "checkbox.h"
#include "container.h"
#include "pushbutton.h"
#include "radiobutton.h"
#include "slider.h"
#include "utils/ustdlib.h"
#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"
#include "includes/common.h"
#include "includes/light_sensor_task.h"
#include "includes/filter_util.h"
#include "includes/display_task.h"
#include "includes/button_task.h"

/*-----------------------------------------------------------*/
/*
 * The binary semaphore used by the switch ISR & task.
 */

extern SemaphoreHandle_t xIC2MasterSemaphore;
extern SemaphoreHandle_t xSampleLightSemaphore;

extern uint32_t g_ui32SysClock;
extern bool day;


/* Event bits */
#define EVENT_HIGH_THRESHOLD (1 << 0)
#define EVENT_LOW_THRESHOLD (1 << 1)
#define EVENT_BTN_TOGGLE (1 << 2)
#define LOW_THRESHOLD 5
EventGroupHandle_t xEventGroup;
/*
 * Global variable to log the last GPIO button pressed.
 */
volatile static uint32_t g_pui32ButtonPressed = NULL;
extern SemaphoreHandle_t xButton1Semaphore;
bool button = false;

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvLightSensorTask(void *pvParameters);
// static void prvDISPTask(void *pvParameters);
static void SignalSampling(TimerHandle_t timer);
void xOptIntHandler(void);

/* Handles the timer interrupt and signals when the read/write task is completed */
void xTimerHandler(void);

/*-----------------------------------------------------------*/

void vCreateLightSensorTask(void)
{
    /* Configure the button to generate interrupts. */
    prvConfigureButton();
    // Set event bits based on thresholds
    xEventGroup = xEventGroupCreate();
    if (xEventGroup == NULL)
    {
        UARTprintf("Failed to create Event Group\n");
        return;
    }
    xTaskCreate(prvLightSensorTask,
                "Light Sensor Sensing",
                1024,
                NULL,
                tskIDLE_PRIORITY+1,
                NULL);
    TimerHandle_t timer = xTimerCreate(
        "Light Sensor Sensing",
        pdMS_TO_TICKS(200),
        pdTRUE,
        0,
        SignalSampling);

    if (xTimerStart(timer, 0) != pdPASS)
    {
        UARTprintf("Failed to start Light Sensor Timer\n");
    }
}
/*-----------------------------------------------------------*/

static void prvLightSensorTask(void *pvParameters)
{
    // Wait for sensor to power up (important!)

    // Now initialize the OPT3001 sensor
    sensorOpt3001Init();

    struct AMessage xMessage;

    bool success;
    uint16_t rawData = 0;
    float convertedLux = 0;

    float filterBuffer[FILTER_SIZE] = {0};
    int filterIndex = 0;
    float filterSum = 0;
    float filteredLux = 0;

    // Test that sensor is set up correctly
    // UARTprintf("Testing OPT3001 Sensor:\n");
    success = sensorOpt3001Test();

    // stay here until sensor is working
    while (!success)
    {
        vTaskDelay(pdMS_TO_TICKS(100)); // Cooperative delay
        UARTprintf("Test Failed, Trying again\n");
        success = sensorOpt3001Test();
    }

    // Loop Forever
    while (1)
    {
        if (xSemaphoreTake(xSampleLightSemaphore, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            // sampling
            success = sensorOpt3001Read(&rawData);
            if (success)
            {
                sensorOpt3001Convert(rawData, &convertedLux);
                filteredLux = MovingAverageFilter(filterBuffer, &filterIndex, &filterSum, FILTER_SIZE, convertedLux);

                //Set event bits based on thresholds
                if (convertedLux > HIGH_THRESHOLD)
                {
                    xEventGroupSetBits(xEventGroup, EVENT_HIGH_THRESHOLD);
                }
                else if (convertedLux < LOW_THRESHOLD)
                {
                    xEventGroupSetBits(xEventGroup, EVENT_LOW_THRESHOLD);
                }
                // add to queue (both raw and filtered values)
                xMessage.ulTimeStamp = xTaskGetTickCount();
                xMessage.uFiltered = filteredLux;
                xMessage.uRaw = convertedLux;
                if (filteredLux > 40)
                    day = true;
                else
                    day = false;
                //UARTprintf("Lux: %d\n",  xMessage.uRaw);
                if (xQueueSend(xLightQueue, (void *)&xMessage, (TickType_t)0) == pdPASS)
                {
                    //UARTprintf("Data sent to queue: %d\n", (int)convertedLux);
                }
                else
                {
                    //UARTprintf("Error LIGHT: Failed to send data to the queue\n");
                }
            }
        }
    }
}

static void SignalSampling(TimerHandle_t timer)
{
    xSemaphoreGive(xSampleLightSemaphore);
}


void xOptIntHandler(void) {
    BaseType_t xOPTTaskWoken = pdFALSE;

    /* Read the PORT P interrupt status to find the cause of the interrupt. */
    uint32_t ui32Status = GPIOIntStatus(GPIO_PORTM_BASE, true);

    /* Clear the interrupt. */
    GPIOIntClear(GPIO_PORTM_BASE, ui32Status);
    //GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, 0);
    xSemaphoreGiveFromISR(xSampleLightSemaphore, &xOPTTaskWoken);
    portYIELD_FROM_ISR(xOPTTaskWoken);
}

// void xI2CHandler(void)
// {
//     BaseType_t xSignalTaskWoken = pdFALSE;

//     // Clear interrupt
//     I2CMasterIntClear(I2C0_BASE);

//     // Only give the semaphore when the I2C bus is idle (transfer finished)
//     if (!I2CMasterBusy(I2C0_BASE))
//     {
//         xSemaphoreGiveFromISR(xIC2MasterSemaphore, &xSignalTaskWoken);
//         portYIELD_FROM_ISR(xSignalTaskWoken);
//     }
// }
