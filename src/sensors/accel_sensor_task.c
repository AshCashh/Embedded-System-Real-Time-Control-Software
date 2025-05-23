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
#include "includes/accel_sensor_task.h"
#include "includes/light_sensor_task.h"
#include "includes/filter_util.h"
#include "includes/display_task.h"
#include "includes/button_task.h"
#include "drivers/bmi160.h"
#include <math.h>

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

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvAccelTask(void *pvParameters);
// static void prvDISPTask(void *pvParameters);

/* Handles the timer interrupt and signals when the read/write task is completed */
void xTimerHandler(void);
void xBMI160DataReadyHandler(void);

/*-----------------------------------------------------------*/

void vCreateAccelTask(void)
{
    /* Configure the button to generate interrupts. */
    //prvConfigureButton();

    xTaskCreate(prvAccelTask,
                "Accel Task",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                NULL);
}

static void prvAccelTask(void *pvParameters)
{
    UARTprintf("[*] Starting Acceleration Task\n");
    if (!sensorBMI160Init())
    {
        UARTprintf("[!] BMI160 Initilisation Failed\n");
    }
    UARTprintf("Initialisation Completed\n");
    SysCtlDelay(pdMS_TO_TICKS(200));

    UARTprintf("[*] Running Tests...\n");
    if (!sensorBMI160Test())
    {
        UARTprintf("Test Failed\n");
    }

    uint8_t status = 0;
    readI2C(0x69, 0x1B, &status);
    UARTprintf("Status: 0x%02X\n", status);

    uint8_t rawData[20];
    static uint32_t lastTick = 0;
    static uint32_t sampleCounter = 0;


    while (1)
    {
        if (xSemaphoreTake(xSampleAccelSemaphore, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            // UARTprintf("    Reading sensor...\n");
            if (!sensorBMI160Read(rawData))
            {
                UARTprintf("[!] Error Reading\n");
            }
            // uint32_t currentTick = xTaskGetTickCount();
            //     sampleCounter++;
            // if (sampleCounter >= 100)  // Log every 20 samples (~every 200 ms at 100 Hz)
            // {
            //     if (lastTick != 0) {
            //         uint32_t delta = currentTick - lastTick;
            //         float frequency = (1000.0f * sampleCounter) / delta;  // ticks in ms
            //         float avgInterval = (float)delta / sampleCounter;
            //         UARTprintf("Avg interval: %d ms, approx %d Hz\n", (int)avgInterval, (int)frequency);
            //     }
            //     lastTick = currentTick;
            //     sampleCounter = 0;
            // }


            // UARTprintf("RAW: %02X %02X %02X %02X %02X %02X\n", rawData[0], rawData[1], rawData[2], rawData[3], rawData[4], rawData[5]);
            int16_t acc_x = (int16_t)((rawData[1] << 8) | rawData[0]);
            int16_t acc_y = (int16_t)((rawData[3] << 8) | rawData[2]);
            int16_t acc_z = (int16_t)((rawData[5] << 8) | rawData[4]);


            //UARTprintf("X: %d, Y: %d, X: %d\n", acc_x, acc_y, acc_z);
            // float accelX = acc_x / 16384.0f; // Convert to g
            // float accelY = acc_y / 16384.0f; // Convert to g
            // float accelZ = acc_z / 16384.0f; // Convert to g
            float accelX = acc_x / 16384.0f * 9.80665; // Convert to SI m/s
            float accelY = acc_y / 16384.0f * 9.80665; // Convert to SI m/s
            float accelZ = (acc_z / 16384.0f * 9.80665); // Convert to SI m/s and cancel out gravity


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
            float avgAbsAccel = (fabs(filteredX) + fabs(filteredY) + fabs(fabs(filteredZ) - 9.81)) / 3.0f;
            UARTprintf("Acceleration: %d.%d\n", (int)avgAbsAccel,(int)(avgAbsAccel * 100) % 100);
        //     UARTprintf("Filtered Accel: X: %d, Y: %d, Z: %d\n",
        //    (int)(filteredX * 1000),
        //    (int)(filteredY * 1000),
        //    (int)(filteredZ * 1000));
        }
    }
}

void xI2CHandler(void)
{
    BaseType_t xSignalTaskWoken = pdFALSE;

    // Clear interrupts
    I2CMasterIntClear(I2C2_BASE);

    // Only give the semaphore when the I2C bus is idle (transfer finished)
    if (!I2CMasterBusy(I2C2_BASE))
    {
        xSemaphoreGiveFromISR(xIC2MasterSemaphore, &xSignalTaskWoken);
        portYIELD_FROM_ISR(xSignalTaskWoken);
    }
}

void xBMI160DataReadyHandler(void) {
    BaseType_t xSignalTaskWoken = pdFALSE;
    // static uint32_t lastTick = 0;
    // uint32_t now = xTaskGetTickCount();
    // UARTprintf("delta=%d\n", now, now - lastTick);
    // lastTick = now;
    GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_3);
    xSemaphoreGiveFromISR(xSampleAccelSemaphore, &xSignalTaskWoken);
}