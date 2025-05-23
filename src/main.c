/*
 * semaphore_example
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
 * This project provides a simple demonstration on how to control GPIO as both
 * inputs and outputs and how to use a binary semaphore to defer ISR processing
 * to an individual task to keep the ISR processing very light.  The four LEDs
 * on the EK-TM4C1294XL are configured with the standard PinoutSet() function.
 * The buttons are used to control which LED is on.  The default is LED D1.
 * Pressing SW1 moves the lit LED down by one and pressing SW2 moves it up by
 * one.  Both switches will wrap around.
 *
 * main() creates one binary semaphore and one task.  It then starts the
 * scheduler.
 *
 * The binary semaphore is used to keep the LED task in a blocked state until
 * an interrupt fires from a switch input.
 *
 * The LED task configures the buttons, initializes the LEDs, and starts the
 * task which will handle the processing for the input switches to toggle the
 * LEDs.
 *
 * This example uses UARTprintf for output of UART messages.  UARTprintf is not
 * a thread-safe API and is only being used for simplicity of the demonstration
 * and in a controlled manner.
 *
 * Open a terminal with 115,200 8-N-1 to see the output for this demo.
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>


/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "inc/hw_sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "driverlib/uart.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "driverlib/i2c.h"
#include "drivers/opt3001.h"

#include "includes/accel_sensor_task.h"
#include "includes/light_sensor_task.h"
#include "includes/display_task.h"
#include "includes/common.h"
/*-----------------------------------------------------------*/

/* The system clock frequency. */
uint32_t g_ui32SysClock;

volatile uint32_t g_ui32TimeStamp = 0;
/* Global for binary semaphore shared between tasks. */
SemaphoreHandle_t xButton1Semaphore = NULL;
SemaphoreHandle_t xButton2Semaphore = NULL;
SemaphoreHandle_t xIC2MasterSemaphore = NULL;
SemaphoreHandle_t xSampleLightSemaphore = NULL;
SemaphoreHandle_t xSampleAccelSemaphore = NULL;

/* Set up the clock and pin configurations to run this example. */
static void prvSetupHardware( void );


/* This function sets up UART0 to be used for a console to display information */
static void prvConfigureUART(void);
static void prvConfigureI2C(void); // configures I2C for sensor communication
static void prvBMI160DataReady(void);

QueueHandle_t xStructQueue = NULL; // Define the variable here
/*
 * Queue used to send and receive pointers to struct AMessage structures.
 */
QueueHandle_t xPointerQueue = NULL;
EventGroupHandle_t xEventGroup = NULL;
/*-----------------------------------------------------------*/

int main( void )
{
    /* Prepare hardware */
    prvSetupHardware();
    /* Create the event group */
    xEventGroup = xEventGroupCreate();
    UARTprintf("Starting System\n");
    if (xEventGroup == NULL)
    {
        UARTprintf("Failed to create Event Group\n");
    
    }
    
    /* Create the queue used to send complete struct AMessage structures.  This can
    also be created after the schedule starts, but care must be task to ensure
    nothing uses the queue until after it has been created. */
    xStructQueue = xQueueCreate(
        /* The number of items the queue can hold. */
        mainQUEUE_LENGTH,
        /* Size of each item is big enough to hold the
        whole structure. */
        sizeof(AMessage));
    

    /* Create the queue used to send pointers to struct AMessage structures. */
    xPointerQueue = xQueueCreate(
        /* The number of items the queue can hold. */
        mainQUEUE_LENGTH,
        /* Size of each item is big enough to hold only a
        pointer. */
        sizeof(AMessage));

    if ((xStructQueue == NULL) || (xPointerQueue == NULL))
    {
    }

    /* Create the binary semaphore used to synchronize the button ISR and the
     * button processing task. */
    xButton1Semaphore = xSemaphoreCreateBinary();
    xButton2Semaphore = xSemaphoreCreateBinary();
    xIC2MasterSemaphore = xSemaphoreCreateBinary();
    xSampleLightSemaphore = xSemaphoreCreateBinary();
    xSampleAccelSemaphore = xSemaphoreCreateBinary();



    if ( xButton1Semaphore != NULL && xButton2Semaphore != NULL && xIC2MasterSemaphore != NULL && xSampleLightSemaphore != NULL)
    {
        /* Configure application specific hardware and initialize the task thread. */
        // vCreateLightSensorTask();
        UARTprintf("Creating Tasks...\n");
        vCreateAccelTask();

        //vCreateDisplayTask();

        /* Start the tasks. */
        vTaskStartScheduler();
        UARTprintf("    Tasks Created\n");
    }
    else {
        UARTprintf("Semaphore creation failed\n");
    }

    /* If all is well, the scheduler will now be running, and the following
    line will never be reached.  If the following line does execute, then
    there was insufficient FreeRTOS heap memory available for the idle and/or
    timer tasks to be created.  See the memory management section on the
    FreeRTOS web site for more details. */
    for( ;; );
}
/*-----------------------------------------------------------*/

// SMBus Interrupt for PORT P Pin 2 (OPT_INT)
static void prvConfigSMBusINT(void) {

    // Enable GPIO port for the INT pin
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);

    // Configure pull-up resistor?
    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_6);
    GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // trigger on the falling edge
    GPIOIntTypeSet(GPIO_PORTM_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);

    // enable GPIOP interrupt
    GPIOIntEnable(GPIO_PORTM_BASE, GPIO_PIN_6);

    IntEnable(INT_GPIOM);

    // enable interrupts
    IntMasterEnable();
}


// config BMI160 data ready interrupt on Port P Pin 3
static void prvBMI160DataReady(void) {
    // Enable GPIO port for the INT pin
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

    // Configure pull-up resistor?
    GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTP_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // trigger on the falling edge
    GPIOIntTypeSet(GPIO_PORTP_BASE, GPIO_PIN_3, GPIO_FALLING_EDGE);
    GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_3);
    // enable GPIOP interrupt
    GPIOIntEnable(GPIO_PORTP_BASE, GPIO_PIN_3);

    IntEnable(INT_GPIOP3);

    // enable interrupts
    IntMasterEnable();
    
}

// config UART
static void prvConfigureUART(void)
{
    /* Enable GPIO port A which is used for UART0 pins.
     * TODO: change this to whichever GPIO port you are using. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    /* Configure the pin muxing for UART0 functions on port A0 and A1.
     * This step is not necessary if your part does not support pin muxing.
     * TODO: change this to select the port/pin you are using. */
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    /* Enable UART0 so that we can configure the clock. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    /* Use the internal 16MHz oscillator as the UART clock source. */
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    /* Select the alternate (UART) function for these pins.
     * TODO: change this to select the port/pin you are using. */
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Initialize the UART for console I/O. */
    UARTStdioConfig(0, 9600, 16000000);
    SysCtlDelay(g_ui32SysClock); // ~1 second delay at 120MHz to ensure UART initialises before using UARTprintf
}

// config I2C
static void prvConfigureI2C(void) {
    //
    // The I2C0 peripheral must be enabled before use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    //
    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    // This step is not necessary if your part does not support pin muxing.
    //
    GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);

    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);

    // Enable I2C0 master interrupt generation
    I2CMasterIntEnable(I2C2_BASE);  // Enables interrupt generation by I2C0 hardware
    
    // Enable I2C0 interrupt in the NVIC
    IntEnable(INT_I2C2);
}

static void prvConfigureHWTimer(void)
{
    /* The Timer 0 peripheral must be enabled for use. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    /* Configure Timer 0 in full-width periodic mode. */
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    /* Set the Timer 0A load value to run at 10 Hz. */
    TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock / 10);

    /* Configure the Timer 0A interrupt for timeout. */
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    /* Enable the Timer 0A interrupt in the NVIC. */
    IntEnable(INT_TIMER0A);

    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();

    //
    // Start the timer used in this example Task
    // You may need change where this timer is enabled
    //
    TimerEnable(TIMER0_BASE, TIMER_A);
}

static void prvSetupHardware( void )
{
    /* Run from the PLL at configCPU_CLOCK_HZ MHz. */
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
            SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
            SYSCTL_CFG_VCO_240), configCPU_CLOCK_HZ);

    /* Configure device pins. */
    PinoutSet(false, false);
    prvConfigureUART();
    prvConfigureI2C();
    prvBMI160DataReady();
    // prvConfigSMBusINT();
    //prvConfigureHWTimer();
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    IntMasterDisable();
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
    /* This function will be called by each tick interrupt if
        configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
        added here, but the tick hook is called from an interrupt context, so
        code must not attempt to block, and only the interrupt safe FreeRTOS API
        functions can be used (those that end in FromISR()). */

    /* Only the full demo uses the tick hook so there is no code is
        executed here. */
}
/*-----------------------------------------------------------*/


void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    IntMasterDisable();
    for( ;; );
}
/*-----------------------------------------------------------*/

void *malloc( size_t xSize )
{
    /* There should not be a heap defined, so trap any attempts to call
    malloc. */
    IntMasterDisable();
    for( ;; );
}
/*-----------------------------------------------------------*/


