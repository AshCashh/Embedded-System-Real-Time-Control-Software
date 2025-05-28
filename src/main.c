/*
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
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "driverlib/timer.h"
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
#include "includes/display_task.h"

#include "includes/accel_sensor_task.h"
#include "includes/light_sensor_task.h"
#include "includes/common.h"

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

#include "variables.h"
// Motor lib
#include <motorlib.h>
motorcontrol_t motor_ctrl;
extern void HallSensorHandler(void);
extern void vCreateMotorTask(void);
static void prvConfigureHallInts(void);
static void prvConfigurePIDTimer(void);
extern void xPIDTimerHandler(void);

static void prvConfigureADCInts(void);
static void prvConfigureCurrentTimer(void);
/*-----------------------------------------------------------*/

SemaphoreHandle_t xPIDTimerSemaphore = NULL;
SemaphoreHandle_t xCountMutex = NULL;
SemaphoreHandle_t xEstop = NULL;
SemaphoreHandle_t xEstopAcknowledge = NULL;
SemaphoreHandle_t xPowerMotorCalcsemaphore = NULL;
/*-----------------------------------------------------------*/

/* The system clock frequency. */
uint32_t g_ui32SysClock;

/* Global for binary semaphore shared between tasks. */
SemaphoreHandle_t xButton1Semaphore = NULL;
SemaphoreHandle_t xButton2Semaphore = NULL;
SemaphoreHandle_t xIC2MasterSemaphore = NULL;
SemaphoreHandle_t xSampleLightSemaphore = NULL;
SemaphoreHandle_t xSampleAccelSemaphore = NULL;
SemaphoreHandle_t xEmergencyStop = NULL;

SemaphoreHandle_t xI2CMutex = NULL;
SemaphoreHandle_t xEmergencyMutex = NULL;

extern SemaphoreHandle_t xSemaphoreTimer0;

extern tDisplay sContext;
/* Set up the clock and pin configurations to run this example. */
static void prvSetupHardware(void);

/* This function sets up UART0 to be used for a console to display information */
static void prvConfigureUART(void);
static void prvConfigureI2C(void); // configures I2C for sensor communication
static void prvBMI160DataReady(void);
static void prvDisplayInit(void);
void clearI2CBus(void);

/*
 * Queue used to send and receive pointers to struct AMessage structures.
 */
QueueHandle_t xPointerQueue = NULL;
QueueHandle_t xLightQueue = NULL;
QueueHandle_t xAccelQueue = NULL;

EventGroupHandle_t xEventGroup = NULL;
/*-----------------------------------------------------------*/

int main(void)
{
    /* Prepare hardware */
    /* Create the event group */
    // xEventGroup = xEventGroupCreate();
    prvSetupHardware();
    UARTprintf("[S]     Starting System\n");
    clearI2CBus();
    // if (xEventGroup == NULL)
    // {
    //     UARTprintf("Failed to create Event Group\n");

    // }

    /* Create the queue used to send complete struct AMessage structures.  This can
    also be created after the schedule starts, but care must be task to ensure
    nothing uses the queue until after it has been created. */
    xLightQueue = xQueueCreate(
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

    /* Create the queue used to send pointers to struct AMessage structures. */
    xAccelQueue = xQueueCreate(
        /* The number of items the queue can hold. */
        mainQUEUE_LENGTH,
        /* Size of each item is big enough to hold only a
        pointer. */
        sizeof(AMessage));

    if ((xLightQueue == NULL) || (xPointerQueue == NULL) || (xAccelQueue == NULL))
    {
        UARTprintf("Queue creation failed\n");
    }

    /* Create the binary semaphore used to synchronize the button ISR and the
     * button processing task. */
    xButton1Semaphore = xSemaphoreCreateBinary();
    xPIDTimerSemaphore = xSemaphoreCreateBinary();
    xEstop = xSemaphoreCreateBinary();
    xEstopAcknowledge = xSemaphoreCreateBinary();
    xPowerMotorCalcsemaphore = xSemaphoreCreateBinary();
    xCountMutex = xSemaphoreCreateMutex();
    motor_ctrl.mutex = xSemaphoreCreateMutex();
    motor_ctrl.pwm = 25;
    motor_ctrl.period_value = 50;
    motor_ctrl.duty_value = PWM_TO_DUTY(motor_ctrl.period_value, motor_ctrl.pwm);
    motor_ctrl.brake = false;

    // MOTOR
    /* Configure motor pins */
    /* Configure ADC1 with ISENCE pins */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
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
           !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE) ||
           !SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1))
        ;
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
    /* Set-up adc interrupts for current measurements */
    prvConfigureADCInts();
    /* Set-up interrupts for hall sensors */
    prvConfigureHallInts();

    xButton2Semaphore = xSemaphoreCreateBinary();
    xIC2MasterSemaphore = xSemaphoreCreateBinary();
    xSampleLightSemaphore = xSemaphoreCreateBinary();
    xSampleAccelSemaphore = xSemaphoreCreateBinary();
    xSemaphoreTimer0 = xSemaphoreCreateBinary();
    xEmergencyStop = xSemaphoreCreateBinary();
    xI2CMutex = xSemaphoreCreateMutex();
    xEmergencyMutex = xSemaphoreCreateMutex();

    if (xButton1Semaphore != NULL && xButton2Semaphore != NULL && xIC2MasterSemaphore != NULL && xSampleLightSemaphore != NULL && xI2CMutex != NULL && xEmergencyStop != NULL && xEmergencyMutex != NULL)
    {
        taskENTER_CRITICAL();
        /* Motor Tasks*/

        prvConfigurePIDTimer();
        /* Configure application specific hardware and initialize the task thread. */
        vCreateAccelTask();
        vCreateDisplayTask();
        vCreateLightSensorTask();
        /* Start the tasks and timer running. */
        vCreateMotorTask();
        taskEXIT_CRITICAL();
        vTaskStartScheduler();
        UARTprintf("    Tasks Created\n");
    }
    else
    {
        UARTprintf("Semaphore creation failed\n");
    }

    /* If all is well, the scheduler will now be running, and the following
    line will never be reached.  If the following line does execute, then
    there was insufficient FreeRTOS heap memory available for the idle and/or
    timer tasks to be created.  See the memory management section on the
    FreeRTOS web site for more details. */
    for (;;)
        ;
}
/*-----------------------------------------------------------*/

void clearI2CBus(void)
{
    // Force SDA and SCL GPIO control
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
        ;

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Simulate 9 clock pulses on SCL to recover stuck slave
    for (int i = 0; i < 9; i++)
    {
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, 0);          // SCL low
        SysCtlDelay(g_ui32SysClock / 100000);                  // ~10us
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, GPIO_PIN_5); // SCL high
        SysCtlDelay(g_ui32SysClock / 100000);
    }

    // Generate a STOP condition: SDA goes high while SCL is high
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0); // SDA low
    SysCtlDelay(g_ui32SysClock / 100000);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, GPIO_PIN_5); // SCL high
    SysCtlDelay(g_ui32SysClock / 100000);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, GPIO_PIN_4); // SDA high

    // Restore I2C pin function
    GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);
}

// SMBus Interrupt for PORT P Pin 2 (OPT_INT)
static void prvConfigSMBusINT(void)
{

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

// // config BMI160 data ready interrupt on Port P Pin 3
// static void prvBMI160DataReady(void) {
//     // Enable GPIO port for the INT pin
//     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

//     // Configure pull-up resistor?
//     GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_4);
//     GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

//     // trigger on the falling edge
//     GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
//     GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_4);
//     // enable GPIOP interrupt
//     GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_4);

//     IntEnable(INT_GPIOD);
// }

// config UART
static void prvConfigureUART(void)
{
    /* Enable GPIO port A which is used for UART0 pins.
     * TODO: change this to whichever GPIO port you are using. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    /* Configure the pin muxing for UART0 functions on port A0 and A1.
     * This step is not necessary if your part does not support pin muxing.
     * TODO: change this to select the port/pin you are using. */
    // GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    /* Enable UART0 so that we can configure the clock. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    /* Use the internal 16MHz oscillator as the UART clock source. */
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    /* Select the alternate (UART) function for these pins.
     * TODO: change this to select the port/pin you are using. */
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_1);

    /* Initialize the UART for console I/O. */
    UARTStdioConfig(0, 9600, 16000000);
    SysCtlDelay(g_ui32SysClock); // ~1 second delay at 120MHz to ensure UART initialises before using UARTprintf
}

static void prvConfigureI2C(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);

    // Set pins to I2C
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);

    // pull-up
    GPIOPadConfigSet(GPIO_PORTN_BASE, GPIO_PIN_4 | GPIO_PIN_5,
                     GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);
    I2CMasterIntEnable(I2C2_BASE);
    IntEnable(INT_I2C2);
}

void prvConfigureHWTimer(void)
{
    /* The Timer 0 peripheral must be enabled for use. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    /* Configure Timer 0 in full-width periodic mode. */
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);

    /* Set the Timer 0A load value to run at 10 Hz. */
    TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock / 30);

    /* Configure the Timer 0A interrupt for timeout. */
    TimerIntRegister(TIMER0_BASE, TIMER_A, xTimerHandler);

    /* Enable the Timer 0A interrupt in the NVIC. */
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();

    //
    // Start the timer used in this example Task
    // You may need change where this timer is enabled
    //
    TimerEnable(TIMER0_BASE, TIMER_A);
}
/*-----------------------------------------------------------*/

static void prvDisplayInit(void)
{
    //
    // The FPU should be enabled because some compilers will use floating-
    // point registers, even for non-floating-point code.  If the FPU is not
    // enabled this will cause a fault.  This also ensures that floating-
    // point operations could be added to this application and would work
    // correctly and use the hardware floating-point unit.  Finally, lazy
    // stacking is enabled for interrupt handlers.  This allows floating-
    // point instructions to be used within interrupt handlers, but at the
    // expense of extra stack usage.
    //
    FPUEnable();
    FPULazyStackingEnable();

    //
    // Initialize the display driver.
    //
    Kentec320x240x16_SSD2119Init(configCPU_CLOCK_HZ);

    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

    SetStartTime(16, 50, 0, "2025-10-01");
    //
    // Initialize the touch screen driver and have it route its messages to the
    // widget tree.
    //
    TouchScreenInit(configCPU_CLOCK_HZ);
    TouchScreenCallbackSet(WidgetPointerMessage);
}

static void prvSetupHardware(void)
{

    /* Run from the PLL at configCPU_CLOCK_HZ MHz. */
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240),
                                            configCPU_CLOCK_HZ);

    /* Configure device pins. */
    PinoutSet(false, false);
    prvDisplayInit();
    prvConfigureUART();
    prvConfigureI2C();
    // prvBMI160DataReady();
    // prvConfigSMBusINT();
    // prvConfigureHWTimer();
    prvConfigureHWTimer(); // timer 0 A
}
/*-----------------------------------------------------------*/
static void prvConfigureHallInts(void)
{

    /* Configure GPIO ports to trigger an interrupt on rising/falling or both edges. */
    /* set interrupts on Hall sensor pins */
    GPIOIntTypeSet(
        GPIO_PORTM_BASE,
        GPIO_PIN_3,
        GPIO_BOTH_EDGES);
    GPIOIntTypeSet(
        GPIO_PORTH_BASE,
        GPIO_PIN_2,
        GPIO_BOTH_EDGES);
    GPIOIntTypeSet(
        GPIO_PORTN_BASE,
        GPIO_PIN_2,
        GPIO_BOTH_EDGES);
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
    /* Int priority maximum */
    IntPrioritySet(INT_GPIOM, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntPrioritySet(INT_GPION, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntPrioritySet(INT_GPIOH, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    /* Clear any prior interrupt flags. */
    GPIOIntClear(HALLA);
    GPIOIntClear(HALLB);
    GPIOIntClear(HALLC);

    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
}
static void prvConfigureADCInts(void)
{
    /* Configure ADC1 to trigger an interrupt on conversion complete. */
    UARTprintf("Configuring ADC1 interrupts\n");
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1))
    {
        // Wait for ADC1 to be ready
    }
    // Configure GPIOE pins as analog inputs
    // ain0 = PE3, ain1 = PD7
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_7);

    // Disable the sequencer before configuration
    ADCSequenceDisable(ADC1_BASE, 1);

    // Configure ADC1 Sequencer 1 (SS1) with processor trigger
    ADCSequenceConfigure(ADC1_BASE, 1, ADC_TRIGGER_TIMER, 0);

    // Step 0: AIN0 (PE3)
    ADCSequenceStepConfigure(ADC1_BASE, 1, 0, ADC_CTL_CH0);
    // Step 1: AIN4 (PE7), with IE and END
    ADCSequenceStepConfigure(ADC1_BASE, 1, 1, ADC_CTL_CH4 | ADC_CTL_IE | ADC_CTL_END);

    ADCSequenceEnable(ADC1_BASE, 1);
    ADCIntClear(ADC1_BASE, 1);
    ADCIntRegister(ADC1_BASE, 1, ADC1IntHandler);
    ADCIntEnable(ADC1_BASE, 1);
    IntEnable(INT_ADC1SS1);

    UARTprintf("ADC1 interrupts configured\n");
    // Configure and start Timer3A
    prvConfigureCurrentTimer();
    UARTprintf("timer3 interrupts configured\n");
}

static void prvConfigureCurrentTimer(void)
{
    /* Use Timer 3A in full width periodic mode at 160hz */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

    /* configure to be periodic*/
    TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER3_BASE, TIMER_A, (g_ui32SysClock / ADC_CURRENT_FREQ)); // 6.25 ms

    /* Configure the Timer 3A interrupt for timeout. */
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    // Configure Timer0A to trigger ADC at timeout
    TimerControlTrigger(TIMER3_BASE, TIMER_A, true);

    // Enable the timer
    TimerEnable(TIMER3_BASE, TIMER_A);
}
static void prvConfigurePIDTimer(void)
{
    /* Use Timer 2A in full width periodic mode at 100hz */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER2_BASE, TIMER_A, (g_ui32SysClock / PID_FREQUENCY)); // 10 ms
    /* Configure the Timer 2A interrupt for timeout. */
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    /* Enable the Timer 2A interrupt in the NVIC. */
    IntEnable(INT_TIMER2A);
    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
    /* Register the Timer 2A interrupt handler. */
    TimerIntRegister(TIMER2_BASE, TIMER_A, xPIDTimerHandler);
    /* Start the timer used in this example Task */
    TimerEnable(TIMER2_BASE, TIMER_A);
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
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
    for (;;)
        ;
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
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

void vApplicationTickHook(void)
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

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    UARTprintf("[!] STACK OVERFLOW in task: %s\n", pcTaskName);
    IntMasterDisable();
    for (;;)
        ;
}
/*-----------------------------------------------------------*/

void *malloc(size_t xSize)
{
    /* There should not be a heap defined, so trap any attempts to call
    malloc. */
    IntMasterDisable();
    for (;;)
        ;
}
/*-----------------------------------------------------------*/
