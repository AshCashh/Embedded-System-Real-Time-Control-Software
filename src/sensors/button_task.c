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

extern SemaphoreHandle_t xButton1Semaphore;
extern SemaphoreHandle_t xButton2Semaphore;
/*
 * Global variable to log the last GPIO button pressed.
 */
volatile static uint32_t g_pui32ButtonPressed = NULL;

/*
 * Hardware configuration for the buttons SW1 and SW2 to generate interrupts.
 */

void prvConfigureButton(void)
{
    /* Initialize the LaunchPad Buttons. */
    ButtonsInit();

    /* Configure both switches to trigger an interrupt on a falling edge. */
    GPIOIntTypeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_FALLING_EDGE);

    /* Enable the interrupt for LaunchPad GPIO Port in the GPIO peripheral. */
    GPIOIntEnable(BUTTONS_GPIO_BASE, ALL_BUTTONS);

    /* Enable the Port F interrupt in the NVIC. */
    IntEnable(INT_GPIOJ);

    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
}

void xButtonsHandler(void)
{
    BaseType_t xLEDTaskWoken;

    uint32_t ui32Status;

    /* Initialize the xLEDTaskWoken as pdFALSE.  This is required as the
     * FreeRTOS interrupt safe API will change it if needed should a
     * context switch be required. */
    xLEDTaskWoken = pdFALSE;

    /* Read the buttons interrupt status to find the cause of the interrupt. */
    ui32Status = GPIOIntStatus(BUTTONS_GPIO_BASE, true);

    if ((ui32Status & USR_SW1) == USR_SW1)
    {
        g_pui32ButtonPressed = USR_SW1;
        xSemaphoreGiveFromISR(xButton1Semaphore, &xLEDTaskWoken);

        // Set the toggle event bit
        xEventGroupSetBitsFromISR(xEventGroup, EVENT_BTN_TOGGLE, &xLEDTaskWoken);
    }

    /* Clear the interrupt. */
    GPIOIntClear(BUTTONS_GPIO_BASE, ui32Status);

    /* Debounce the input with 200ms filter */
    if ((xTaskGetTickCount() - g_ui32TimeStamp) > 150)
    {
        /* Log which button was pressed to trigger the ISR. */
        if ((ui32Status & USR_SW1) == USR_SW1)
        {
            g_pui32ButtonPressed = USR_SW1;
            /* Give the semaphore to unblock prvProcessSwitchInputTask.  */
            xSemaphoreGiveFromISR(xButton1Semaphore, &xLEDTaskWoken);
        }

        /* This FreeRTOS API call will handle the context switch if it is
         * required or have no effect if that is not needed. */
        portYIELD_FROM_ISR(xLEDTaskWoken);
    }

    /* Update the time stamp. */
    g_ui32TimeStamp = xTaskGetTickCount();
}