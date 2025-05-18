/*
 * hello
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
 * This motor test project provides an example of how to use the motor library
 * with a platformio / freeRTOS project. The main script initialises the hall 
 * sensor interrupt, which run the update_motor function. The program also launches 
 * a task that initialises the motors before ramping the speed from 10% to 100%.
 * Once the speed reaches 100%, the motor is stopped and the program ends.
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
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "variables.h"
// Motor lib
#include <motorlib.h>

#include "driverlib/timer.h"

motorcontrol_t motor_ctrl;

/*-----------------------------------------------------------*/
extern void HallSensorHandler(void);
/* The system clock frequency. */
uint32_t g_ui32SysClock;

/* Set up the hardware ready to run this demo. */
static void prvSetupHardware( void );

/* This function sets up UART0 to be used for a console to display information
 * as the example is running. */
static void prvConfigureUART(void);

/* API to trigger the 'Hello world' task. */
extern void vCreateMotorTask( void );

static void prvConfigureHallInts( void );

static void prvConfigureHWTimer(void);
/*-----------------------------------------------------------*/
/*
    Initialises sempahores
*/
SemaphoreHandle_t xButtonSemaphore = NULL;
SemaphoreHandle_t xHallSemaphore = NULL;

int main( void )
{   
    xButtonSemaphore = xSemaphoreCreateBinary();
    xHallSemaphore = xSemaphoreCreateBinary();
    motor_ctrl.mutex = xSemaphoreCreateMutex();
    motor_ctrl.duty_value = 15;
    motor_ctrl.period_value = 50;
    motor_ctrl.brake = false;
    /* Prepare the hardware to run this demo. */
    prvSetupHardware();

    /* Create the Hello task to output a message over UART. */
    vCreateMotorTask();
    prvConfigureHWTimer();

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    /* If all is well, the scheduler will now be running, and the following
    line will never be reached.  If the following line does execute, then
    there was insufficient FreeRTOS heap memory available for the idle and/or
    timer tasks to be created.  See the memory management section on the
    FreeRTOS web site for more details. */
    for( ;; );
}
/*-----------------------------------------------------------*/
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
}
/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
    /* Run from the PLL at configCPU_CLOCK_HZ MHz. */
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
            SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
            SYSCTL_CFG_VCO_240), configCPU_CLOCK_HZ);

    /* Configure device pins. */
    PinoutSet(false, false);

    /* Configure UART0 to send messages to terminal. */
    prvConfigureUART();
    
    /* Configure motor pins */
    /* Enable GPIO ports for motor phases */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF) ||
           !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG) ||
           !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC) ||
            !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOH) ||
            !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM) ||
            !SysCtlPeripheralReady(SYSCTL_PERIPH_GPION) ||
            !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA) ||
            !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD) ||
            !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    /* Configure phase pins as outputs */
    GPIOPinTypeGPIOOutput(INLA);
    GPIOPinTypeGPIOOutput(INHB);
    GPIOPinTypeGPIOOutput(INLB);
    GPIOPinTypeGPIOOutput(INHC);
    GPIOPinTypeGPIOOutput(INLC);
    GPIOPinTypeGPIOOutput(ENA);
    /* Configure sense pins as inputs */
    GPIOPinTypeGPIOInput(ISENCE_A);
    GPIOPinTypeGPIOInput(ISENCE_B);
    GPIOPinTypeGPIOInput(ISENCE_C);
    /* Configure Hall sensor pins as inputs */
    GPIOPinTypeGPIOInput(HALLA);
    GPIOPinTypeGPIOInput(HALLB);
    GPIOPinTypeGPIOInput(HALLC);
    /* Disable brake */
    GPIOPinWrite(INLC, GPIO_PIN_5);
    /* Drive forwards */
    GPIOPinWrite(INHC, 0);
    /* Set-up interrupts for hall sensors */
    prvConfigureHallInts();


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
static void prvConfigureHallInts( void )
{

    /* Configure GPIO ports to trigger an interrupt on rising/falling or both edges. */
    /* set interrupts on Hall sensor pins */
    GPIOIntTypeSet(
        GPIO_PORTM_BASE,
        GPIO_PIN_3,
        GPIO_BOTH_EDGES
    );
    GPIOIntTypeSet(
        GPIO_PORTH_BASE,
        GPIO_PIN_2,
        GPIO_BOTH_EDGES
    );
    GPIOIntTypeSet(
        GPIO_PORTN_BASE,
        GPIO_PIN_2,
        GPIO_BOTH_EDGES
    );
    /* raise interrupt priority for hallsensorhandler */
    IntPrioritySet(INT_GPIOM, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntPrioritySet(INT_GPION, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntPrioritySet(INT_GPIOH, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    /* Enable the GPIO interrupt for Hall sensor pins. */
    GPIOIntEnable(HALLA);
    GPIOIntEnable(HALLB);
    GPIOIntEnable(HALLC);
    /* Enable the GPIO interrupt handler. */
    GPIOIntRegister(GPIO_PORTM_BASE, HallSensorHandler);
    GPIOIntRegister(GPIO_PORTH_BASE, HallSensorHandler);
    GPIOIntRegister(GPIO_PORTN_BASE, HallSensorHandler);
    /* Enable pullups */
    GPIOPadConfigSet(HALLA,
        GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(HALLB,
        GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(HALLC,
        GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    /* Clear any prior interrupt flags. */
    GPIOIntClear(HALLA);
    GPIOIntClear(HALLB);
    GPIOIntClear(HALLC);



    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
}

/*-----------------------------------------------------------*/
static void prvConfigureHWTimer(void)
{
    /* The Timer 0 peripheral must be enabled for use. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    /* Configure Timer 0 in full-width periodic mode. */
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    /* Set the Timer 0A load value to run at generall 10 hz. check the variable function to know*/
    TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock/COUNT_REFRESH_RATE_HZ)); // 100 ms

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


