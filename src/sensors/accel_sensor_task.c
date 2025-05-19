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
#include "driver_lib/bmi160.h"

/*-----------------------------------------------------------*/
/*
 * The binary semaphore used by the switch ISR & task.
 */

extern SemaphoreHandle_t xIC2MasterSemaphore;
extern SemaphoreHandle_t xSampleAccelSemaphore;

extern uint32_t g_ui32SysClock;

tContext sContext;
// Moving average filter variables
#define FILTER_SIZE 10
static float filterBufferX[FILTER_SIZE] = {0};
static float filterBufferY[FILTER_SIZE] = {0};
static float filterBufferZ[FILTER_SIZE] = {0};
static int filterIndex = 0;
static float filterSumX = 0;
static float filterSumY = 0;
static float filterSumZ = 0;
// not sure about this
struct bmi160_t s_bmi160;
struct bmi160_accel_t accelData;


// Structure for acceleration data
typedef struct {
    uint32_t ulTimeStamp;
    float accelX;
    float accelY;
    float accelZ;
    float avgAbsAccel;
} AccelMessage;

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvAccelTask(void *pvParameters);
// static void prvDISPTask(void *pvParameters);
static void SignalSampling(TimerHandle_t timer);

/* Handles the timer interrupt and signals when the read/write task is completed */
void xTimerHandler(void);

/*-----------------------------------------------------------*/

void vCreateAccelTask(void)
{
    /* Configure the button to generate interrupts. */
    prvConfigureButton();

    xTaskCreate(prvAccelTask,
                "Accel Task",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                NULL);
    TimerHandle_t timer = xTimerCreate(
        "Light Sensor Sensing",
        pdMS_TO_TICKS(100),
        pdTRUE,
        0,
        SignalSampling);

    if (xTimerStart(timer, 0) != pdPASS)
    {
        UARTprintf("Failed to start Light Sensor Timer\n");
    }
}
/*-----------------------------------------------------------*/

static void prvAccelTask(void *pvParameters)
{

    UARTprintf("Accel Task Started\n");
    // Initialize the BMI160 sensor
    if (bmi160_init(&s_bmi160) == -1) {
        UARTprintf("[!] BMI160 Initilisation Failed\n");
    }
    UARTprintf("Initialisation Completed\n");

    AccelMessage accelMessage;

    while (1)
    {
        UARTprintf("Pizza\n");
        if (xSemaphoreTake(xSampleAccelSemaphore, portMAX_DELAY) == pdTRUE)
        {
            // Read acceleration data
            if (bmi160_read_accel_xyz(&accelData) == 0)
            {
                float accelX = accelData.x / 16384.0f; // Convert to g
                float accelY = accelData.y / 16384.0f; // Convert to g
                float accelZ = accelData.z / 16384.0f; // Convert to g

                // Update moving average filters
                filterSumX -= filterBufferX[filterIndex];
                filterSumY -= filterBufferY[filterIndex];
                filterSumZ -= filterBufferZ[filterIndex];

                filterBufferX[filterIndex] = accelX;
                filterBufferY[filterIndex] = accelY;
                filterBufferZ[filterIndex] = accelZ;

                filterSumX += accelX;
                filterSumY += accelY;
                filterSumZ += accelZ;

                filterIndex = (filterIndex + 1) % FILTER_SIZE;

                float filteredX = filterSumX / FILTER_SIZE;
                float filteredY = filterSumY / FILTER_SIZE;
                float filteredZ = filterSumZ / FILTER_SIZE;

                // Calculate average absolute acceleration
                float avgAbsAccel = (fabs(filteredX) + fabs(filteredY) + fabs(filteredZ)) / 3.0f;
                UARTprintf("Filtered Accel: X: %.2f, Y: %.2f, Z: %.2f\n", filteredX, filteredY, filteredZ);
            }
        }
    }
}

static void SignalSampling(TimerHandle_t timer)
{
    xSemaphoreGive(xSampleAccelSemaphore);
}

void xI2CHandler(void)
{
    BaseType_t xSignalTaskWoken = pdFALSE;

    // Clear interrupt
    I2CMasterIntClear(I2C0_BASE);

    // Only give the semaphore when the I2C bus is idle (transfer finished)
    if (!I2CMasterBusy(I2C0_BASE))
    {
        xSemaphoreGiveFromISR(xIC2MasterSemaphore, &xSignalTaskWoken);
        portYIELD_FROM_ISR(xSignalTaskWoken);
    }
}
