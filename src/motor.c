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

extern motorcontrol_t motor_ctrl;

volatile float latest_rpm;
volatile uint32_t last_tick = 0;
uint32_t last_hall_update = 0;
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
extern SemaphoreHandle_t xPowerMotorCalcsemaphore;
QueueHandle_t xMotorRPMQueue;
QueueHandle_t xPowerQueue;

/*
 * Global variable to log the last GPIO button pressed.
 */
volatile static uint32_t g_pui32ButtonPressed = NULL;

void HallSensorHandler(void);

void ADC1IntHandler(void);
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
static void prvCurrentReadTask(void *pvParameters);

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

    xMotorRPMQueue = xQueueCreate(QUEUE_LENGTH, sizeof(AMessage));
    xPowerQueue = xQueueCreate(QUEUE_LENGTH, sizeof(AMessage));
    if (xMotorRPMQueue == NULL)
    {
        // Handle error
        UARTprintf("Failed to create motor RPM queue\n");
    }
    
    xTaskCreate(prvButtonTask,
                "ButtonTask",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 3,
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
    xTaskCreate(prvCurrentReadTask,
                "CurrentRead",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
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

static void prvCurrentReadTask(void *pvParameters)
{
    uint32_t adcValues[2];
    float voltage0, voltage4, voltageE;
    float current0, current4, currentE;

    uint32_t counter = 0;

    float filtered_current0, filtered_current4, filtered_currentE;

    float current0_avg, current4_avg, currentE_avg;
    float current0_calc_error;
    current0_avg = 0.0f;
    current4_avg = 0.0f;
    currentE_avg = 0.0f;

    float power0,power1,powerE;
    float power_raw0, power_raw1, power_rawE;

    UARTprintf("Current Read Task Started\n");

    for (;;)
    {
        if (xSemaphoreTake(xPowerMotorCalcsemaphore, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            //  UARTprintf("before trigger\n");
            // Read conversion results
            ADCSequenceDataGet(ADC1_BASE, 1, adcValues);
            voltage0 = (float)(adcValues[0] / ADC_MAX_VALUE) * VREF;
            voltage4 = (float)(adcValues[1] / ADC_MAX_VALUE) * VREF;
            voltageE = (voltage0 + voltage4) / 2.0f;
            // UARTprintf("%d,%d,%d\n", (int)(1000*voltage0), (int)(1000*voltage4),(int)(1000*voltageE));
            // // Convert to current: I = (V/2 - 1.65) / (Rshunt × Gain)
            current0 = (((VREF / 2) - voltage0) / (GAIN * RSHUNT)); // A
            current4 = (((VREF / 2) - voltage4) / (GAIN * RSHUNT)); // A
            current0_calc_error = (((VREF / 2) - voltage0-0.02f) / (GAIN * RSHUNT));
            // // I1 +I2 +I3 = 0 because the motor is a three-phase system
            // // Current E is the estimated 3rd current
            // // I3 = i(I1+I2)
            currentE = -(current0_calc_error + current4)+Motor_INEFFICIENCY;
            // UARTprintf("%d,%d,%d\n", (int)(1000*current0), (int)(1000*current4), (int)(1000*(currentE)));
            current0_avg += current0;
            current4_avg += current4;
            currentE_avg += currentE;
            counter++;
            if (counter >= ADC_CURRENT_SAMPLES)
            {
                // Calculate the average current
                filtered_current0 = AMPS_TO_MILLIAMPS((current0_avg / ADC_CURRENT_SAMPLES));
                filtered_current4 = AMPS_TO_MILLIAMPS((current4_avg / ADC_CURRENT_SAMPLES));
                filtered_currentE = AMPS_TO_MILLIAMPS((currentE_avg / ADC_CURRENT_SAMPLES));
                // print to uart
                power0 = (POWER_CALCULATE(filtered_current0))/1000;
                power1 = (POWER_CALCULATE(filtered_current4))/1000;
                powerE = (POWER_CALCULATE(filtered_currentE))/1000; // in mWatts
                /* Unfilitered power */
                power_raw0 = (POWER_CALCULATE(current0))/1000;
                power_raw1 = (POWER_CALCULATE(current4))/1000;
                power_rawE = (POWER_CALCULATE(currentE))/1000; // in mWatts

                // UARTprintf("%d\n", (int)Power);
                // UARTprintf("%d,%d,%d\n", (int)filtered_current0, (int)filtered_current4, (int)filtered_currentE);
                // UARTprintf("%d,%d,%d\n", (int)power0, (int)power1, (int)powerE);
                // Reset the counter and averages
                counter = 0;
                current0_avg = 0;
                current4_avg = 0;
                currentE_avg = 0;
                /* Send message to queue */
                AMessage xMessage;
                xMessage.ulTimeStamp = xTaskGetTickCount();
                xMessage.uRaw = (uint32_t)(power_raw0 * 1000); // Store as milliWatts
                xMessage.uFiltered = (uint32_t)(power0 * 1000); // Store as milliWatts
                if (xQueueSend(xPowerQueue, (void *)&xMessage, (TickType_t)0) != pdTRUE)
                {
                    // Handle queue full error
                    UARTprintf("Failed to send power data to queue\n");
                }
            }

        }
        // Use current0 and current4 in control logic or print/log
    }
}

static void prvEmergencyCheckTask(void *pvParameters)
{
    for (;;)
    {
        if (xSemaphoreTake(xEstop, portMAX_DELAY) == pdTRUE)
        {
            if (xSemaphoreTake(motor_ctrl.mutex, portMAX_DELAY) == pdTRUE)
            {

                motor_ctrl.estop = true;
                // UARTprintf("\nMotor disabled\n");
                motor_ctrl.motor_enabled = false;
                motor_ctrl.pwm = 1;
                motor_ctrl.stall_counter = STALL_VAL + 1;
                motor_ctrl.target_rpm = 0;
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
                motor_ctrl.estop = false;
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
                enableMotor();
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
                if (xSemaphoreTake(motor_ctrl.mutex, portMAX_DELAY) == pdTRUE)
                {
                    motor_ctrl.estop = false;
                    UARTprintf("Estop disengaged\n");
                    // if ((motor_ctrl.rpm - BUTTON_RPM_INCREMENT) <= 1)
                    // {
                    //     xSemaphoreGive(xEstop);
                    // }
                    // else
                    // {
                    //     motor_ctrl.target_rpm -= BUTTON_RPM_INCREMENT;
                    // }
                    xSemaphoreGive(motor_ctrl.mutex);
                }
                g_pui32ButtonPressed = USR_SW1;
            }
            else if ((ui32ButtonStatus & USR_SW2) == USR_SW2)
            {
                if (xSemaphoreTake(motor_ctrl.mutex, portMAX_DELAY) == pdTRUE)
                {
                    motor_ctrl.estop = true;
                    UARTprintf("Estop engaged\n");
                    // if (motor_ctrl.motor_enabled == false)
                    // {
                    //     xSemaphoreGive(xEstopAcknowledge);
                    // }
                    // // prevents the duty cycle from going out of range
                    // // Prevents the pwm from going past 100%
                    // else if ((motor_ctrl.pwm + BUTTON_DUTY_INCREMENT) >= 96)
                    // {
                    //     // Safety feature
                    //     UARTprintf("\nDUTY VALUE MAXED OUT\n");

                    //     // Sends the motor down to a slightly safer value
                    //     motor_ctrl.pwm = 90;
                    // }
                    // else
                    // {
                    //     motor_ctrl.target_rpm += BUTTON_RPM_INCREMENT; // Increase target RPM
                    // }
                    // UARTprintf("Duty value %d\n", motor_ctrl.duty_value);
                    xSemaphoreGive(motor_ctrl.mutex);
                }
                g_pui32ButtonPressed = USR_SW2;
            }
        }
    }
}

static void prvMotorPIDTask(void *parameters)
{
    float rpm_prev = 0.0f;
    float error_prev = 0.0f;
    float integral = 0.0f;

    /* RPM moving‐average state */
    static float rpm_buffer[MOVING_AVERAGE_SAMPLES] = {0};
    static uint32_t rpm_index = 0;
    static float rpm_sum = 0.0f;

    /* Acceleration moving‐average state */
    static float accel_buffer[MOVING_AVERAGE_SAMPLES] = {0};
    static uint32_t accel_index = 0;
    static float accel_sum = 0.0f;
    static bool local_estop = false;
    /* Ramp RPM to limit acceleration exceeding */
    static float ramped_target_rpm = 0.0f;
    /* RPM limit vars */
    const float max_accel_delta = (MAX_ACCELERATION_RPMS * dt);
    float max_decel_delta = (MAX_DECELERATION_RPMS * dt);
    /* consider only regular deceleration for now */
    float raw_rpm = 0;
    float local_target_rpm;
    uint32_t local_period;
    prvMotorStart();
    
    for (;;)
    {
        if (xSemaphoreTake(xPIDTimerSemaphore, pdMS_TO_TICKS(2000)) != pdTRUE)
            continue;

        taskENTER_CRITICAL();

        /* clear stale data */
        if ((last_hall_update - xTaskGetTickCount()) > pdMS_TO_TICKS(800))
        {
            raw_rpm = 0;
        }
        raw_rpm = latest_rpm;
        local_target_rpm = motor_ctrl.target_rpm;
        local_period = motor_ctrl.period_value;
        local_estop = motor_ctrl.estop;
        taskEXIT_CRITICAL();
        max_decel_delta = (MAX_DECELERATION_RPMS * dt);
        if (local_estop)
        {
            max_decel_delta = (ESTOP_DECELERATION_RPMS * dt);

        }
        rpm_sum -= rpm_buffer[rpm_index];
        rpm_buffer[rpm_index] = raw_rpm;
        rpm_sum += raw_rpm;
        rpm_index = (rpm_index + 1) % MOVING_AVERAGE_SAMPLES;
        float rpm = rpm_sum / (float)MOVING_AVERAGE_SAMPLES;

        float acceleration = (rpm - rpm_prev) * PID_FREQUENCY;
        rpm_prev = rpm;

        accel_sum -= accel_buffer[accel_index];
        accel_buffer[accel_index] = acceleration;
        accel_sum += acceleration;
        accel_index = (accel_index + 1) % MOVING_AVERAGE_SAMPLES;
        float avg_acceleration = accel_sum / (float)MOVING_AVERAGE_SAMPLES;
        /* clamp local target rpm to prevent overshooting acceleration */

        if ((local_target_rpm - ramped_target_rpm) > max_accel_delta)
        {
            ramped_target_rpm += max_accel_delta;
        }
        else if ((local_target_rpm - ramped_target_rpm) < -max_decel_delta)
        {
            ramped_target_rpm -= max_decel_delta;
        }
        else
        {
            ramped_target_rpm = local_target_rpm;
            /* use actual rpm to as reference now */
        } 

        /* PID loop */
        float error = ramped_target_rpm - rpm;
        integral += error * dt;
        float derivative = (error - error_prev) / dt;
        error_prev = error;
        /* Clamp input to PWM duty cycle */
        float u = clamp(Kp * error + Ki * integral + Kd * derivative, 2.0f, 100.0f);
        uint32_t local_duty = PWM_TO_DUTY(local_period, u);

        bool need_disable = false;
        if (xSemaphoreTake(motor_ctrl.mutex, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            motor_ctrl.rpm = rpm;
            motor_ctrl.pwm = u;
            motor_ctrl.duty_value = local_duty;
            xSemaphoreGive(motor_ctrl.mutex);
        }


        setDuty(local_duty);
        if (need_disable)
            disableMotor();

        /* Send RPM data to queue */
        AMessage xMessage;
        xMessage.ulTimeStamp = xTaskGetTickCount();
        xMessage.uRaw = (uint32_t)(ramped_target_rpm * 1000); // Store as milliRPM
        xMessage.uFiltered = (uint32_t)(rpm * 1000); // Store as milliRPM
        if (xQueueSend(xMotorRPMQueue, (void *)&xMessage, (TickType_t)0) != pdTRUE)
        {
            // Handle queue full error
            UARTprintf("Failed to send RPM data to queue\n");
        }
        UARTprintf("RPM: %d, Target: %d, AvgAccel: %d, Ramped Target RPM: %d\n",
                   (int)rpm,
                   (int)local_target_rpm,
                   (int)avg_acceleration,
                   (int)ramped_target_rpm);
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
    last_hall_update = xTaskGetTickCountFromISR();
    uint32_t tick_delta = last_hall_update - last_tick;
    last_tick = last_hall_update;
    float minute_delta = TICKS_TO_MINUTES(tick_delta);
    latest_rpm = 1 / (COUNT_PER_REVOLUTION * minute_delta);
    int tmp[3] = {0, 0, 0};
    getHallSensorValues(tmp);
    updateMotor(tmp[0], tmp[1], tmp[2]);
    // xSemaphoreGiveFromISR(xPowerMotorCalcsemaphore, &xMotorTaskWoken);
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
        motor_ctrl.pwm = 50;
        motor_ctrl.target_rpm = 1000;
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

void ADC1IntHandler(void)
{
    /*
     * ADC1 interrupt handler
     * This function is called when the ADC1 interrupt is triggered.
     * It clears the interrupt and updates the motor phase based on the hall sensor values.
     */
    // Clear the ADC interrupt
    ADCIntClear(ADC1_BASE, 1);
    // Trigger the current read task
    xSemaphoreGiveFromISR(xPowerMotorCalcsemaphore, NULL);
}