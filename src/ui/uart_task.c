// /*
//  * led_task
//  *
//  * Copyright (C) 2022 Texas Instruments Incorporated
//  *
//  *
//  *  Redistribution and use in source and binary forms, with or without
//  *  modification, are permitted provided that the following conditions
//  *  are met:
//  *
//  *    Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  *
//  *    Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in the
//  *    documentation and/or other materials provided with the
//  *    distribution.
//  *
//  *    Neither the name of Texas Instruments Incorporated nor the names of
//  *    its contributors may be used to endorse or promote products derived
//  *    from this software without specific prior written permission.
//  *
//  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  *
//  */

// /******************************************************************************
//  *
//  * vLEDTask turns on the initial LED, configurations the buttons, and creates
//  * prvProcessSwitchInputTask which handles processing the ISR result to adjust
//  * the LED per the button inputs.
//  *
//  * prvProcessSwitchInputTask uses semaphore take to wait until it receives a
//  * semaphore from the button ISR.  Once it does, then it processes the button
//  * pressed.  Each time the SW1 or SW2 button is pressed, the LED index is
//  * updated based on which button has been pressed and the corresponding LED
//  * lights up.
//  *
//  * When either user switch SW1 or SW2 on the EK-TM4C1294XL is pressed, an
//  * interrupt is generated and the switch pressed is logged in the global
//  * variable g_pui32ButtonPressed.  Then the binary semaphore is given to
//  * prvProcessSwitchInputTask before yielding to it.  This is an example of
//  * using binary semaphores to defer ISR processing to a task.
//  *
//  */

// /* Standard includes. */
// #include <stdio.h>
// #include <stdint.h>
// #include <stdbool.h>

// /* Kernel includes. */
// #include "FreeRTOS.h"
// #include "task.h"
// #include "semphr.h"
// #include "timers.h"
// #include "event_groups.h"
// #include "queue.h"

// /* Hardware includes. */
// #include "inc/hw_ints.h"
// #include "inc/hw_memmap.h"
// #include "driverlib/gpio.h"
// #include "driverlib/interrupt.h"
// #include "driverlib/sysctl.h"
// #include "drivers/rtos_hw_drivers.h"
// #include "utils/uartstdio.h"
// #include "driverlib/timer.h"
// #include "drivers/i2cOptDriver.h"
// #include "driverlib/i2c.h"
// #include "drivers/opt3001.h"

// /* Hardware includes. */
// #include "inc/hw_ints.h"
// #include "inc/hw_memmap.h"
// #include "driverlib/sysctl.h"
// #include "drivers/rtos_hw_drivers.h"
// #include "utils/uartstdio.h"
// #include "inc/hw_nvic.h"
// #include "inc/hw_sysctl.h"
// #include "inc/hw_types.h"
// #include "driverlib/fpu.h"
// #include "driverlib/gpio.h"
// #include "driverlib/flash.h"
// #include "driverlib/sysctl.h"
// #include "driverlib/systick.h"
// #include "driverlib/uart.h"
// #include "driverlib/udma.h"
// #include "driverlib/rom.h"
// #include "driverlib/rom_map.h"
// #include "grlib.h"
// #include "widget.h"
// #include "canvas.h"
// #include "checkbox.h"
// #include "container.h"
// #include "pushbutton.h"
// #include "radiobutton.h"
// #include "slider.h"
// #include "utils/ustdlib.h"
// #include "drivers/Kentec320x240x16_ssd2119_spi.h"
// #include "drivers/touch.h"
// #include "includes/common.h"
// #include "includes/accel_sensor_task.h"
// #include "includes/filter_util.h"
// #include "includes/display_task.h"
// #include "includes/button_task.h"
// #include "drivers/bmi160.h"
// #include <math.h>
// #include "includes/uart_task.h"

// /*-----------------------------------------------------------*/
// /*
//  * The binary semaphore used by the switch ISR & task.
//  */


// static void prvUARTTask(void *pvParameters);
// /*
//  * The tasks as described in the comments at the top of this file.
//  */

// // static void prvDISPTask(void *pvParameters);


// /*-----------------------------------------------------------*/

// void vCreateUARTTask(void)
// {
//     /* Configure the button to generate interrupts. */
//     // prvConfigureButton();

//     xTaskCreate(prvUARTTask,
//                 "UART Task",
//                 configMINIMAL_STACK_SIZE, // Increased stack size for BMI160 operations
//                 NULL,
//                 tskIDLE_PRIORITY,
//                 NULL);
// }

// static void prvUARTTask(void *pvParameters)
// {
//     UARTprintf("UART TASK STARTED\n");
//     char printBuffer[UART_PRINT_MAX_STRING_LENGTH];

//     while (1)
//     {
//         if (xQueueReceive(xUARTQueue, &printBuffer, portMAX_DELAY) == pdTRUE)
//         {
//             UARTprintf("%s", printBuffer);
//         }
//     }
// }
