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
#include "math.h"
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
#define AVERAGE_SAMPLES 60

extern motorcontrol_t motor_ctrl;

volatile uint32_t count = 0;
/*
 * Time stamp global variable.
 */
volatile uint32_t g_ui32TimeStamp = 0;
volatile uint32_t ui32ButtonStatus;

extern volatile uint32_t g_ui32SysClock;

/*Semaphores intialised in main*/
extern SemaphoreHandle_t xButtonSemaphore;
extern SemaphoreHandle_t xPIDTimerSemaphore;
extern SemaphoreHandle_t xCountMutex;
extern SemaphoreHandle_t xEstop;
extern SemaphoreHandle_t xEstopAcknowledge;
QueueHandle_t xMotorTimestampQueue;

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
static void prvMotorPIDTask(void *pvParameters);
static void prvMotorStart(void);

static void prvEmergencyCheckTask(void *pvParameters);
static void prvEmergencyAckTask(void *pvParameters);

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

    xMotorTimestampQueue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(prvButtonTask,
                "ButtonTask",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    xTaskCreate(prvMotorPIDTask,
                "MotorPID",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);

    xTaskCreate(prvEmergencyCheckTask,
                "EmergencyCheck",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 4,
                NULL);
    xTaskCreate(prvEmergencyAckTask,
                "EmergencyAck",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 3,
                NULL);
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


static void prvEmergencyCheckTask(void *pvParameters)
{
    for (;;)
    {
        if (xSemaphoreTake(xEstop, portMAX_DELAY) == pdTRUE)
        {
            if (xSemaphoreTake(motor_ctrl.mutex, portMAX_DELAY) == pdTRUE)
            {

                motor_ctrl.Estop = true;
                // UARTprintf("\nMotor disabled\n");
                motor_ctrl.motor_enabled = false;
                motor_ctrl.pwm = 1;
                motor_ctrl.stall_counter = STALL_VAL + 1;
                disableMotor();
                xSemaphoreGive(motor_ctrl.mutex);
            }
            UARTprintf("Emergency stop Triggered\n");
        }
    }
}

static void prvEmergencyAckTask(void *pvParameters)
{
    for (;;)
    {
        if (xSemaphoreTake(xEstopAcknowledge, portMAX_DELAY) == pdTRUE)
        {
            if (xSemaphoreTake(motor_ctrl.mutex, portMAX_DELAY) == pdTRUE)
            {
                motor_ctrl.Estop = false;
                motor_ctrl.motor_enabled = true;
                motor_ctrl.stall_counter = 0;
                motor_ctrl.pwm = 50; // 50%
                motor_ctrl.target_rpm = 1000;
                getHallSensorValues(motor_ctrl.hall_sensor_values);
                updateMotor(motor_ctrl.hall_sensor_values[0],
                            motor_ctrl.hall_sensor_values[1],
                            motor_ctrl.hall_sensor_values[2]);

                motor_ctrl.duty_value = PWM_TO_DUTY(motor_ctrl.period_value, motor_ctrl.pwm);
                setDuty(motor_ctrl.duty_value);
                xSemaphoreGive(motor_ctrl.mutex);
            }
            UARTprintf("Emergency stop acknowledged\n");
        }
    }
}



static void prvButtonTask(void *pvParameters)
{
    /*
        * Button task
        * This task is responsible for handling the button presses and updating
        * the motor speed and direction accordingly. It uses a semaphore to
        * synchronize with the button interrupt handler.
        * This is primarily for testing purposes in the place of an Actual UI
    */
    for (;;)
    {
        // only runs if the timer indicates and update has occured
        if (xSemaphoreTake(xButtonSemaphore, portMAX_DELAY) == pdPASS)
        {
            // UARTprintf("Button task started\n");
            if ((ui32ButtonStatus & USR_SW1) == USR_SW1)
            {
                xSemaphoreGive(xEstop);
                // // Have as little processing as possible within the locked mutex to prevent unnecesary slow down
                // if (xSemaphoreTake(motor_ctrl.mutex, portMAX_DELAY) == pdTRUE)
                // {
                //     if ((motor_ctrl.rpm - BUTTON_RPM_INCREMENT) <= 1)
                //     {
                //         // Safety feature
                //         UARTprintf("\nDuty value too low, setting to disabling motor\n");
                //         // UARTprintf("\nMotor disabled\n");
                //         motor_ctrl.motor_enabled = false;
                //         motor_ctrl.pwm = 1;
                //         motor_ctrl.target_rpm = 0;
                //         motor_ctrl.stall_counter = STALL_VAL+1;
                //         disableMotor();
                //     }
                //     else
                //     {
                //         motor_ctrl.target_rpm -= BUTTON_RPM_INCREMENT;
                //     }
                //     // UARTprintf("Duty value %d\n", motor_ctrl.duty_value);
                //     xSemaphoreGive(motor_ctrl.mutex);
                // }
                g_pui32ButtonPressed = USR_SW1;
            }
            else if ((ui32ButtonStatus & USR_SW2) == USR_SW2)
            {
                xSemaphoreGive(xEstopAcknowledge);
                // if (xSemaphoreTake(motor_ctrl.mutex, portMAX_DELAY) == pdTRUE)
                // {
                //     if (motor_ctrl.motor_enabled == false)
                //     {
                //         /* Renables the motors within this task as it often requires rapid reaction */
                //         UARTprintf("\nMotor Re-enabled after stall\n");
                //         motor_ctrl.motor_enabled = true;
                //         motor_ctrl.stall_counter = 0;
                //         motor_ctrl.pwm = 30; //30%
                //         motor_ctrl.target_rpm = 1000;
                //         getHallSensorValues(motor_ctrl.hall_sensor_values);
                //         updateMotor(motor_ctrl.hall_sensor_values[0],
                //                     motor_ctrl.hall_sensor_values[1],
                //                     motor_ctrl.hall_sensor_values[2]);
                        
                //         motor_ctrl.duty_value = PWM_TO_DUTY(motor_ctrl.period_value, motor_ctrl.pwm);   
                //         setDuty(motor_ctrl.duty_value);
                //         enableMotor();
                //     }
                //     // prevents the duty cycle from going out of range
                //     //Prevents the pwm from going past 100%
                //     else if ((motor_ctrl.pwm + BUTTON_DUTY_INCREMENT) >= 96)
                //     {
                //         // Safety feature
                //         UARTprintf("\nDUTY VALUE MAXED OUT\n");

                //         //Sends the motor down to a slightly safer value
                //         motor_ctrl.pwm = 90;
                //     }
                //     else
                //     {
                //         motor_ctrl.target_rpm += BUTTON_RPM_INCREMENT; // Increase target RPM
                //     }
                //     // UARTprintf("Duty value %d\n", motor_ctrl.duty_value);
                //     xSemaphoreGive(motor_ctrl.mutex);
                // }
                g_pui32ButtonPressed = USR_SW2;
            }
        }
    }
}

static void prvMotorPIDTask( void* parameters )
{
    uint32_t hall_int_count = 0;
    float rpm = 0.0f;
    float rpm_prev = 0.0f; // Previous RPM for acceleration calculation
    float target_rpm = 0.0f;
    float acceleration = 0.0f; // Acceleration in RPM/s
    float error = 0.0f;
    float u = 0.0f; // Control signal
    float integral = 0.0f; // Integral term
    float derivative = 0.0f; // Derivative term
    float error_prev = 0.0f; // Previous error for derivative calculation
    prvMotorStart(); // Start the motor and initialize the control
    for (;;)
    {
        if (xSemaphoreTake(xPIDTimerSemaphore, pdMS_TO_TICKS(2000)) == pdTRUE)
        {
            /* enter critical section to get count and leave */
            taskENTER_CRITICAL();
            hall_int_count = count;
            count = 0; // reset count
            target_rpm = motor_ctrl.target_rpm; // Get target RPM from motor control struct
            taskEXIT_CRITICAL();
            rpm = count_to_rpm(hall_int_count);
            error = target_rpm - rpm;
            integral = integral + error * dt; // Integral term
            // Prevent integral windup
            derivative = (error - error_prev) / dt; // Derivative term
            /* clamp integral error to avoid windup */
            u = Kp * error + Ki * integral + Kd * derivative; // PID control signal
            error_prev = error; // Update previous error
            // Clamp the control signal to a valid range
            u = clamp(u, 2, 100); // Assuming u is a percentage value (2-100%)
            if (xSemaphoreTake(motor_ctrl.mutex, pdMS_TO_TICKS(20)) == pdTRUE)
            {
                motor_ctrl.rpm = rpm; // Update RPM in motor control struct
                motor_ctrl.pwm = u; // Update PWM value based on control signal
                motor_ctrl.duty_value = PWM_TO_DUTY(motor_ctrl.period_value, motor_ctrl.pwm);
                setDuty(motor_ctrl.duty_value); // Set the duty cycle
                
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
                    motor_ctrl.target_rpm = 0;
                    integral = 0.0f;
                    motor_ctrl.motor_enabled = false;
                    motor_ctrl.stall_counter = STALL_VAL + 1;
                    disableMotor();
                }
                xSemaphoreGive(motor_ctrl.mutex);
            }
            acceleration = (float)(rpm - rpm_prev) / (float)PID_FREQUENCY; // Calculate acceleration in RPM/s
            UARTprintf("%d, %d, %d\n", (int)rpm, (int)target_rpm, (int)acceleration);
            rpm_prev = rpm; // Update previous RPM for next iteration
        }
    }
}

/*-----------------------------------------------------------*/
/* Interrupt handlers */

void HallSensorHandler(void)
{
    /*
    * Hall sensor interrupt handler
    * This function is called when the hall sensor interrupts are triggered.
    * It clears the interrupt and updates the motor phase based on the hall
    */
    /* Get type of interrupt */
    BaseType_t xMotorTaskWoken = pdFALSE;
    uint32_t ui32StatusM = GPIOIntStatus(GPIO_PORTM_BASE, true);
    uint32_t ui32StatusH = GPIOIntStatus(GPIO_PORTH_BASE, true);
    uint32_t ui32StatusN = GPIOIntStatus(GPIO_PORTN_BASE, true);

    /* Clear the interrupt */
    GPIOIntClear(GPIO_PORTM_BASE, ui32StatusM);
    GPIOIntClear(GPIO_PORTH_BASE, ui32StatusH);
    GPIOIntClear(GPIO_PORTN_BASE, ui32StatusN);

    // /* trigger interrupt on port m pin 2*/
    // if (ui32StatusM & GPIO_PIN_3)
    // {
    //     /* give semaphore */
    //     uint32_t timestamp = xTaskGetTickCount();
    //     xQueueSendFromISR(xMotorTimestampQueue, &timestamp, NULL);
    //     portYIELD_FROM_ISR(xMotorTaskWoken);
    // }

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


void xPIDTimerHandler(void)
{
    /*
    * This timer is used to update the motor control calculations
    * It is used to update the RPM and acceleration values
    * It is also used to update the motor control calculations
    */
    /* Clear the hardware interrupt flag for Timer 2A. */
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    /* Initialize xTimerWoken as pdFALSE.  This is required as the
     * FreeRTOS interrupt safe API will change it if needed should a
     * context switch be required. */
    BaseType_t xPIDTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xPIDTimerSemaphore, &xPIDTaskWoken);


    /*give the semaphore*/
    portYIELD_FROM_ISR(xPIDTaskWoken);
}

void prvMotorStart()
{
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
}