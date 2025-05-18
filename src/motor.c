/*
 * hello_task
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
 * The Hello task creates a simple task to handle the UART output for the
 * 'Hello World!' message.  A loop is executed five times with a count down
 * before ending with the self-termination of the task that prints the UART
 * message by use of vTaskDelete.  The loop also includes a one second delay
 * that is achieved by using vTaskDelay.
 *
 * This example uses UARTprintf for output of UART messages.  UARTprintf is not
 * a thread-safe API and is only being used for simplicity of the demonstration
 * and in a controlled manner.
 *
 */

/* Standard includes. */
#include "driverlib/pin_map.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "variables.h"
#include "motorlib.h"

#define BUTTON_DUTY_INCREMENT 5

extern motorcontrol_t motor_ctrl;

static int count = 0;
/*
 * Time stamp global variable.
 */
volatile uint32_t g_ui32TimeStamp = 0;
volatile uint32_t ui32ButtonStatus;

extern volatile uint32_t g_ui32SysClock;

/*Semaphores intialised in main*/
extern SemaphoreHandle_t xButtonSemaphore;
extern SemaphoreHandle_t xCountTimerSemaphore;

/*
 * Global variable to log the last GPIO button pressed.
 */
volatile static uint32_t g_pui32ButtonPressed = NULL;

void HallSensorHandler(void);
/*-----------------------------------------------------------*/

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvMotorTask(void *pvParameters);
static void prvButtonTask(void *pvParameters);
static void prvMotorCalcTask(void *pvParameters);

/*
 * Called by main() to create the Hello print task.
 */
void vCreateMotorTask(void);

static void prvConfigureButton(void);

// void prvConfigureButton(void);

/*
 * Hardware interrupt handlers
 */

/*-----------------------------------------------------------*/

void vCreateMotorTask(void)
{
    /* Create the task as described in the comments at the top of this file.
     *
     * The xTaskCreate parameters in order are:
     *  - The function that implements the task.
     *  - The text name Hello task - for debug only as it is
     *    not used by the kernel.
     *  - The size of the stack to allocate to the task.
     *  - No parameter passed to the task
     *  - The priority assigned to the task.
     *  - The task handle is NULL */

    xTaskCreate(prvMotorTask,
                "MotorTask",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);
    xTaskCreate(prvButtonTask,
                "ButtonTask",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    xTaskCreate(prvMotorCalcTask,
                "MotorCalc",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
}
/*-----------------------------------------------------------*/

static void prvMotorTask(void *pvParameters)
{
    //

    UARTprintf("Motor task started\n");
    if (motor_ctrl.mutex == NULL)
    {
        // Handle error
        UARTprintf("Failed to create mutex\n");
    }

    // configure buttons
    prvConfigureButton();
    if (xSemaphoreTake(motor_ctrl.mutex, portMAX_DELAY) == pdTRUE)
    {
        /* Initialise the motors and set the duty cycle (speed) in microseconds */
        initMotorLib(motor_ctrl.period_value);

        motor_ctrl.duty_value = PWM_TO_DUTY(motor_ctrl.period_value, motor_ctrl.pwm);
        setDuty(motor_ctrl.duty_value);
        motor_ctrl.motor_enabled = true;
        motor_ctrl.stall_counter = 0;
        motor_ctrl.acceleration = 0;
        xSemaphoreGive(motor_ctrl.mutex);
    }
    else
    {
        // Handle error
        UARTprintf("Failed to take mutex\n");
    }
    /* start motor phase cycle */
    enableMotor();
    /* Kick start the motor */
    // Do an initial read of the hall effect sensor GPIO lines
    /* read hall sensor gpio lines */
    UARTprintf("Getting hall values\n");
    if (xSemaphoreTake(motor_ctrl.mutex, portMAX_DELAY) == pdTRUE)
    {
        getHallSensorValues(motor_ctrl.hall_sensor_values);
        updateMotor(motor_ctrl.hall_sensor_values[0],
                    motor_ctrl.hall_sensor_values[1],
                    motor_ctrl.hall_sensor_values[2]);
        xSemaphoreGive(motor_ctrl.mutex);
    }
    else
    {
        // Handle error
        UARTprintf("Failed to take mutex\n");
    }

    // give the read hall effect sensor lines to updateMotor() to move the motor
    // one single phase
    // Recommendation is to use an interrupt on the hall effect sensors GPIO lines
    // So that the motor continues to be updated every time the GPIO lines change from high to low
    // or low to high
    // Include the updateMotor function call in the ISR to achieve this behaviour.

    /* Motor test - ramp up the duty cycle from 10% to 100%, than stop the motor */
    // The Values below are before intialisation so the program stops yelling at me, they will never need to be used
    uint32_t rpm = 0, acceleration = 0, pwm = 15, period_value = 100,duty_value = PWM_TO_DUTY(period_value, pwm);
    bool motor_enabled = false;
    for (;;)
    {
        if (xSemaphoreTake(motor_ctrl.mutex, portMAX_DELAY) == pdTRUE)
        {
            // UARTprintf("Motor task mutex taken\n");
            pwm = motor_ctrl.pwm;
            period_value = motor_ctrl.period_value;

            //update duty value
            motor_ctrl.duty_value = PWM_TO_DUTY(period_value, pwm);

            duty_value = motor_ctrl.duty_value;
            
            rpm = motor_ctrl.rpm;
            acceleration = motor_ctrl.acceleration;
            motor_enabled = motor_ctrl.motor_enabled;
            xSemaphoreGive(motor_ctrl.mutex);
        }

        if (motor_enabled)
        {

            if ((0 >= duty_value) || (duty_value <= period_value))
            {
                setDuty(duty_value);
            }
            else
            {
                // additional saftey feature, shouldn't happen, but incase it does
                // stopMotor(1);
                disableMotor();
                UARTprintf("INVALID DUTY_CYCLE\n");
                break;
            }
            UARTprintf("\rDuty cycle: %d  PWM:%d  RPM: %d  Acceleration(RPM/s): %d   ", duty_value,pwm,rpm,acceleration);
        }
        else
        {
            if (xSemaphoreTake(motor_ctrl.mutex, portMAX_DELAY) == pdTRUE)
            {
                if (motor_ctrl.stall_counter > STALL_VAL)
                {
                    UARTprintf("\rMotor disabled");
                    disableMotor();
                }
                xSemaphoreGive(motor_ctrl.mutex);
            }
        }
    }
}
/*-----------------------------------------------------------*/
static void prvConfigureButton(void)
{
    IntMasterDisable();
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
static void prvButtonTask(void *pvParameters)
{
    for (;;)
    {
        // only runs if the timer indicates and update has occured
        if (xSemaphoreTake(xButtonSemaphore, portMAX_DELAY) == pdPASS)
        {
            // UARTprintf("Button task started\n");
            if ((ui32ButtonStatus & USR_SW1) == USR_SW1)
            {
                // Have as little processing as possible within the locked mutex to prevent unnecesary slow down
                if (xSemaphoreTake(motor_ctrl.mutex, portMAX_DELAY) == pdTRUE)
                {
                    if ((motor_ctrl.pwm - BUTTON_DUTY_INCREMENT) <= 1)
                    {
                        // Safety feature
                        UARTprintf("\nDuty value too low, setting to disabling motor\n");
                        // UARTprintf("\nMotor disabled\n");
                        motor_ctrl.motor_enabled = false;
                        motor_ctrl.pwm = 1;
                        motor_ctrl.stall_counter = STALL_VAL+1;
                        disableMotor();
                    }
                    else
                    {
                        motor_ctrl.pwm -= BUTTON_DUTY_INCREMENT;
                    }
                    // UARTprintf("Duty value %d\n", motor_ctrl.duty_value);
                    xSemaphoreGive(motor_ctrl.mutex);
                }
                g_pui32ButtonPressed = USR_SW1;
            }
            else if ((ui32ButtonStatus & USR_SW2) == USR_SW2)
            {
                if (xSemaphoreTake(motor_ctrl.mutex, portMAX_DELAY) == pdTRUE)
                {
                    if (motor_ctrl.motor_enabled == false)
                    {
                        /* Renables the motors within this task as it often requires rapid reaction */
                        taskENTER_CRITICAL();
                        UARTprintf("\nMotor Re-enabled after stall\n");
                        motor_ctrl.motor_enabled = true;
                        motor_ctrl.stall_counter = 0;
                        motor_ctrl.pwm = 15;
                        getHallSensorValues(motor_ctrl.hall_sensor_values);
                        updateMotor(motor_ctrl.hall_sensor_values[0],
                                    motor_ctrl.hall_sensor_values[1],
                                    motor_ctrl.hall_sensor_values[2]);
                        
                        motor_ctrl.duty_value = PWM_TO_DUTY(motor_ctrl.period_value, motor_ctrl.pwm);   
                        setDuty(motor_ctrl.duty_value);
                        enableMotor();
                        taskEXIT_CRITICAL();
                    }
                    // prevents the duty cycle from going out of range
                    //Prevents the pwm from going past 100%
                    else if ((motor_ctrl.pwm + BUTTON_DUTY_INCREMENT) >= 96)
                    {
                        // Safety feature
                        UARTprintf("\nDUTY VALUE MAXED OUT\n");

                        //Sends the motor down to a slightly safer value
                        motor_ctrl.pwm = 90;
                    }
                    else
                    {
                        motor_ctrl.pwm += BUTTON_DUTY_INCREMENT;
                    }
                    // UARTprintf("Duty value %d\n", motor_ctrl.duty_value);
                    xSemaphoreGive(motor_ctrl.mutex);
                }
                g_pui32ButtonPressed = USR_SW2;
            }
        }
    }
}

static void prvMotorCalcTask(void *pvParameters)
{
    // uint32_t last_update = xTaskGetTickCount();
    // counter is rp/100ms -> rp/ms*10 = rps -> rps*60 = rpm
    // counter*600/6 =
    for (;;)
    {
        if (xSemaphoreTake(xCountTimerSemaphore, portMAX_DELAY) == pdTRUE)
        {
            if (xSemaphoreTake(motor_ctrl.mutex, portMAX_DELAY) == pdTRUE)
            {
                taskENTER_CRITICAL();
                uint32_t old_rpm = motor_ctrl.rpm;
                motor_ctrl.rpm = COUNT_TO_RPM(count);
                motor_ctrl.acceleration = (motor_ctrl.rpm - old_rpm) * COUNT_REFRESH_RATE_HZ;
                if ((motor_ctrl.stall_counter < STALL_VAL) && (motor_ctrl.rpm == 0))
                {
                    // UARTprintf("Motor Stalling\n");
                    motor_ctrl.stall_counter++;
                }
                else if (motor_ctrl.rpm > 0)
                {
                    motor_ctrl.stall_counter = 0;
                }
                else if (motor_ctrl.stall_counter >= STALL_VAL)
                {

                    // UARTprintf("Motor Stalled\n");
                    motor_ctrl.motor_enabled = false;
                    motor_ctrl.stall_counter = STALL_VAL+1;
                    disableMotor();
                }

                count = 0;
                taskEXIT_CRITICAL();
                xSemaphoreGive(motor_ctrl.mutex);
            }
            // UARTprintf("IN Task\n");
        }
    }
}
/*-----------------------------------------------------------*/
/* Interrupt handlers */

void HallSensorHandler(void)
{
    /* Get type of interrupt */
    uint32_t ui32StatusM = GPIOIntStatus(GPIO_PORTM_BASE, true);
    uint32_t ui32StatusH = GPIOIntStatus(GPIO_PORTH_BASE, true);
    uint32_t ui32StatusN = GPIOIntStatus(GPIO_PORTN_BASE, true);

    /* Clear the interrupt */
    GPIOIntClear(GPIO_PORTM_BASE, ui32StatusM);
    GPIOIntClear(GPIO_PORTH_BASE, ui32StatusH);
    GPIOIntClear(GPIO_PORTN_BASE, ui32StatusN);

    count++;
    int tmp[3] = {0, 0, 0};
    getHallSensorValues(tmp);
    updateMotor(tmp[0], tmp[1], tmp[2]);

}
void xButtonsHandler(void)
{
    BaseType_t xButtonTaskWoken;
    /* Initialize the xLEDTaskWoken as pdFALSE.  This is required as the
     * FreeRTOS interrupt safe API will change it if needed should a
     * context switch be required. */
    xButtonTaskWoken = pdFALSE;

    /* Read the buttons interrupt status to find the cause of the interrupt. */
    ui32ButtonStatus = GPIOIntStatus(BUTTONS_GPIO_BASE, true);

    /* Clear the interrupt. */
    GPIOIntClear(BUTTONS_GPIO_BASE, ui32ButtonStatus);

    /* Debounce the input with 100ms filter */
    // Can reduce this value to increase response time of button
    // but if too small can lead to debouncing issues
    if ((xTaskGetTickCount() - g_ui32TimeStamp) > 100)
    {
        /* This FreeRTOS API call will handle the context switch if it is
         * required or have no effect if that is not needed. */
        xSemaphoreGiveFromISR(xButtonSemaphore, &xButtonTaskWoken);
        portYIELD_FROM_ISR(xButtonTaskWoken);
    }

    /* Update the time stamp. */
    g_ui32TimeStamp = xTaskGetTickCount();
}
void xTimerHandler(void)
{
    /* Clear the hardware interrupt flag for Timer 0A. */
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    /* Initialize xTimerWoken as pdFALSE.  This is required as the
     * FreeRTOS interrupt safe API will change it if needed should a
     * context switch be required. */
    BaseType_t xCountTaskWoken = pdFALSE;

    
    /*give the semaphore*/
    xSemaphoreGiveFromISR(xCountTimerSemaphore, &xCountTaskWoken);
    portYIELD_FROM_ISR(xCountTaskWoken);
}
