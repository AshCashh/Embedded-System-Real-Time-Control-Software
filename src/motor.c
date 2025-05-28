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
#include "includes/common.h"
#define BUTTON_DUTY_INCREMENT 5

extern motorcontrol_t motor_ctrl;

volatile float latest_rpm;
volatile uint32_t last_tick = 0;
uint32_t last_hall_update = 0;
volatile uint32_t count = 0;
/*
 * Time stamp global variable.
 */
volatile uint32_t g_ui32TimeStamp = 0;
// volatile uint32_t ui32ButtonStatus;

extern volatile uint32_t g_ui32SysClock;

/*Semaphores intialised in main*/
// extern SemaphoreHandle_t xButtonSemaphore;
extern SemaphoreHandle_t xPIDTimerSemaphore;
extern SemaphoreHandle_t xCountMutex;
extern SemaphoreHandle_t xEstop;
extern SemaphoreHandle_t xEstopAcknowledge;
extern SemaphoreHandle_t xPowerMotorCalcsemaphore;

extern QueueHandle_t xMotorRPMQueue;
extern QueueHandle_t xPowerQueue;

/*
 * Global variable to log the last GPIO button pressed.
 */
// volatile static uint32_t g_pui32ButtonPressed = NULL;

extern float LowPassFilter(float previousValue, float newValue, float alpha);
void HallSensorHandler(void);

void ADC1IntHandler(void);
/*-----------------------------------------------------------*/

/*
 * The tasks as described in the comments at the top of this file.
 */
// static void prvMotorTask(void *pvParameters);
// static void prvButtonTask(void *pvParameters);
static void prvMotorPIDTask(void *pvParameters);
static void prvMotorStart(void);

// static void prvEmergencyCheckTask(void *pvParameters);
// static void prvEmergencyAckTask(void *pvParameters);
static void prvCurrentReadTask(void *pvParameters);

/*
 * Called by main() to create the Hello print task.
 */
void vCreateMotorTask(void);

// static void prvConfigureButton(void);

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

    xTaskCreate(prvMotorPIDTask,
                "MotorPID",
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
// static void prvConfigureButton(void)
// {
//     IntMasterDisable();
//     /* Initialize the LaunchPad Buttons. */
//     ButtonsInit();

//     /* Configure both switches to trigger an interrupt on a falling edge. */
//     GPIOIntTypeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_FALLING_EDGE);

//     /* Enable the interrupt for LaunchPad GPIO Port in the GPIO peripheral. */
//     GPIOIntEnable(BUTTONS_GPIO_BASE, ALL_BUTTONS);

//     /* Enable the Port F interrupt in the NVIC. */
//     IntEnable(INT_GPIOJ);

//     /* Enable global interrupts in the NVIC. */
//     IntMasterEnable();
// }

float convert_val_to_current(uint32_t adc_value)
{
    /*
     * Convert the ADC value to current in mA
     * Formula: I = (V/2 - 1.65) / (Rshunt × Gain)
     * where V = adc_value / ADC_MAX_VALUE * VREF
     */
    float voltage = (float)(adc_value / ADC_MAX_VALUE) * VREF;
    float current = (((VREF / 2.f) - voltage) / (GAIN * RSHUNT)); // A
    return current;
}

static void prvCurrentReadTask(void *pvParameters)
{
    /* RPM moving‐average state */
    static float current_1_buffer[MOVING_AVERAGE_SAMPLES] = {0};
    static uint32_t current_index = 0;
    static float current_1_sum = 0.0f;

    /* RPM moving‐average state */
    static float current_2_buffer[MOVING_AVERAGE_SAMPLES] = {0};
    static float current_2_sum = 0.0f;

    /* RPM moving‐average state */
    static float current_e_buffer[MOVING_AVERAGE_SAMPLES] = {0};
    static float current_e_sum = 0.0f;

    float Static_point1, Static_point2;
    Static_point1 = 0.0f; // Initial static point for current difference
    Static_point2 = 0.0f; // Initial static point for current difference

    float Large_Current_1_Average = 0.0f;
    float Large_Current_2_Average = 0.0f;

    uint32_t large_count = 0; // Reset large count for current averaging

    uint32_t adcValues[2];
    float voltage0, voltage4, voltageE;
    float raw_current0, raw_current4, raw_currentE;

    float filtered_current1, filtered_current2, filtered_currentE;

    float power0_filtered, power1_filtered, powerE_filtered;
    float power_raw0, power_raw1, power_rawE;

    filtered_current1 = 0.0f;
    filtered_current2 = 0.0f;
    filtered_currentE = 0.0f;

    float power_raw, power_filtered;

    power_filtered = 0.0f;

    UARTprintf("Current Read Task Started\n");
    AMessage xMessage;

    for (;;)
    {
        if (xSemaphoreTake(xPowerMotorCalcsemaphore, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            //  UARTprintf("before trigger\n");
            // Read conversion results
            ADCSequenceDataGet(ADC1_BASE, 1, adcValues);
            uint32_t ADC_SENSOR, ADC_SENSOR_4;

            // uint32_t ADC_ADJUSMENT = 100; // Adjust this value based on calibration

            ADC_SENSOR = adcValues[0];
            ADC_SENSOR_4 = adcValues[1];

            // UARTprintf("ADC Values: %d, %d,%d\n", ADC_SENSOR, ADC_SENSOR_4,(ADC_SENSOR - ADC_SENSOR_4));

            voltage0 = (((float)ADC_SENSOR) / (float)ADC_MAX_VALUE) * VREF;
            voltage4 = (((float)ADC_SENSOR_4) / (float)ADC_MAX_VALUE) * VREF;
            voltageE = (voltage0 + voltage4) / 2.0f;

            // // Convert to current: I = (V/2 - 1.65) / (Rshunt × Gain)
            raw_current0 = ((((VREF_DIV2)-voltage0) / (GAIN * RSHUNT)) - Static_point1); //+ 0.13f;  // A
            raw_current4 = ((((VREF_DIV2)-voltage4) / (GAIN * RSHUNT)) - Static_point2); //- 0.473f; // A

            // calculate the estimated current E with some error correction
            // current0_calc_error = (((VREF_DIV2)-voltage0 - 0.02f) / (GAIN * RSHUNT));

            // // I1 +I2 +I3 = (motor Inefficiency [constant]) because the motor is a non-perfect three-phase system
            // // Current E is the estimated 3rd current
            // // I3 = -(I1+I2) + Motor_INEFFICIENCY

            raw_currentE = -(raw_current0 + raw_current4);

            // UARTprintf("%d,%d,%d\n", (int)(1000*current0), (int)(1000*current4), (int)(1000*(currentE)));

            power_raw0 = raw_current0 * MOTOR_NORMAL_VOLTAGE; // in Watts
            power_raw1 = raw_current4 * MOTOR_NORMAL_VOLTAGE; // in Watts
            power_rawE = raw_currentE * MOTOR_NORMAL_VOLTAGE; // in Watts

            power_raw = (power_raw0 + power_raw1 + power_rawE); // Average raw power in Watts

            if (large_count > ADC_CURRENT_SAMPLES_AVERAGE_FIX)
            {
                // Reset the large count and averages
                Static_point1 = Static_point1 + (Large_Current_1_Average / (float)ADC_CURRENT_SAMPLES_AVERAGE_FIX);
                Static_point2 = Static_point2 + (Large_Current_2_Average / (float)ADC_CURRENT_SAMPLES_AVERAGE_FIX);
                large_count = 0;
                Large_Current_1_Average = 0.0f;
                Large_Current_2_Average = 0.0f;

            }
            large_count++;

            Large_Current_1_Average = Large_Current_1_Average + raw_current0;
            Large_Current_2_Average = Large_Current_2_Average + raw_current4;

            if (current_index > ADC_CURRENT_SAMPLES)
            {
                raw_current0 = LowPassFilter(filtered_current1, raw_current0, LOW_PASS_FILTER_ALPHA);
                raw_current4 = LowPassFilter(filtered_current2, raw_current4, LOW_PASS_FILTER_ALPHA);
                raw_currentE = LowPassFilter(filtered_currentE, raw_currentE, LOW_PASS_FILTER_ALPHA);
            }

            //
            current_index = (current_index + 1) % ADC_CURRENT_SAMPLES;
            // Calculate the average current

            /*Current 1 (pin0)*/
            current_1_sum -= current_1_buffer[current_index];
            current_1_buffer[current_index] = raw_current0;
            current_1_sum += raw_current0;
            filtered_current1 = current_1_sum / (float)ADC_CURRENT_SAMPLES;

            /*Current 2 (pin4)*/
            current_2_sum -= current_2_buffer[current_index];
            current_2_buffer[current_index] = raw_current4;
            current_2_sum += raw_current4;
            filtered_current2 = current_2_sum / (float)ADC_CURRENT_SAMPLES;

            /*Current E [ESTIMATED]*/
            current_e_sum -= current_e_buffer[current_index];
            current_e_buffer[current_index] = raw_currentE;
            current_e_sum += raw_currentE;
            filtered_currentE = current_e_sum / (float)ADC_CURRENT_SAMPLES;

            power0_filtered = (filtered_current1 * MOTOR_NORMAL_VOLTAGE); // in Watts
            power1_filtered = (filtered_current2 * MOTOR_NORMAL_VOLTAGE); // in Watts
            powerE_filtered = (filtered_currentE * MOTOR_NORMAL_VOLTAGE); // in Watts
            UARTprintf("%d, %d, %d\n", (int)(1000 * filtered_current1), (int)(1000 * filtered_current2), (int)(1000 * filtered_currentE));
            power_filtered = (power0_filtered + power1_filtered + powerE_filtered);

            // UARTprintf("%d,%d\n", (int)(power_filtered*1000), (int)(power_raw*1000));
            xMessage.uFiltered = (uint32_t)(power_filtered * 1000); // Convert to mA
            xMessage.uRaw = (uint32_t)(power_raw * 1000);           // Convert to mA
            xMessage.ulTimeStamp = xTaskGetTickCount();

            if (xQueueSend(xMotorRPMQueue, (void *)&xMessage, (TickType_t)0) == pdPASS)
            {
                // UARTprintf("Current sent: %d\n", xMessage.uRaw);
            }
            else
            {
                // UARTprintf("Error CURRENT: Failed to send data to the queue\n");
            }
        }

        // Use current0 and current4 in control logic or print/log
    }
}

static void prvMotorPIDTask(void *parameters)
{

    /* RPM moving‐average state */
    static float rpm_buffer[MOVING_AVERAGE_SAMPLES] = {0};
    static uint32_t rpm_index = 0;
    static float rpm_sum = 0.0f;

    /* Acceleration moving‐average state */
    static float accel_buffer[MOVING_AVERAGE_SAMPLES] = {0};
    static uint32_t accel_index = 0;
    static float accel_sum = 0.0f;

    /* Ramp RPM to limit acceleration exceeding */
    static float ramped_target_rpm = 0.0f;
    /* RPM limit vars */
    const float max_accel_delta = (MAX_ACCELERATION_RPMS * dt);
    /* consider only regular deceleration for now */
    float max_decel_delta = (MAX_DECELERATION_RPMS * dt);
    float raw_rpm = 0;
    float local_target_rpm;
    uint32_t local_period;
    prvMotorStart();
    /* create message var */
    AMessage xMessage;
    float u;
    float rpm_prev = 0.0f;
    float error = 0.0f;
    float error_prev = 0.0f;
    float integral = 0.0f;
    float derivative = 0.0f;
    for (;;)
    {
        if (xSemaphoreTake(xPIDTimerSemaphore, pdMS_TO_TICKS(2000)) != pdTRUE)
            continue;

        taskENTER_CRITICAL();

        /* clear stale data */
        // if ((last_hall_update - xTaskGetTickCount()) > pdMS_TO_TICKS(800))
        // {
        //     raw_rpm = 0;
        // }
        raw_rpm = latest_rpm;
        local_target_rpm = motor_ctrl.target_rpm;
        local_period = motor_ctrl.period_value;
        /* don't accumulate error if in stop state */
        if (!motor_ctrl.motor_enabled)
        {
            local_target_rpm = 0.0f;
        }
        else
        {
            enableMotor();
        }
        /* if estop set deceleration rate to be estop */
        if (motor_ctrl.Estop)
        {
            max_decel_delta = (ESTOP_DECELERATION_RPMS * dt);
        }
        taskEXIT_CRITICAL();
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
        /* Clamp ramped target to enforce max acceleration relative to actual RPM */
        float delta_rpm = local_target_rpm - rpm;

        if (delta_rpm > max_accel_delta)
        {
            ramped_target_rpm = rpm + max_accel_delta;
        }
        else if (delta_rpm < -max_decel_delta)
        {
            ramped_target_rpm = rpm - max_decel_delta;
        }
        else
        {
            ramped_target_rpm = local_target_rpm;
        }

        /* send rpm in queue */
        xMessage.ulTimeStamp = xTaskGetTickCount();
        xMessage.uFiltered = (uint32_t)(rpm);
        xMessage.uRaw = (uint32_t)(raw_rpm);
        if (xQueueSend(xMotorRPMQueue, (void *)&xMessage, (TickType_t)0) != pdPASS)
            ;
        {
        }
        /* PID loop */
        error = ramped_target_rpm - rpm;
        // if (fabs(ramped_target_rpm - local_target_rpm) < (local_target_rpm * 0.10f)) {setDuty(PWM_TO_DUTY(local_period, u)); continue;} // If the target RPM is within 5% of the local target RPM, skip PID control
        integral += error * dt;                           // Integral term
        derivative = (error - error_prev) / dt;           // Derivative term
        u = Kp * error + Ki * integral + Kd * derivative; // PID control signal
        error_prev = error;                               // Update previous error
        // Clamp the control signal to a valid range
        u = clamp(u, 2, 100); // Assuming u is a percentage value (0-100%)
        uint32_t local_duty = PWM_TO_DUTY(local_period, u);

        // bool need_disable = false;
        if (xSemaphoreTake(motor_ctrl.mutex, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            motor_ctrl.rpm = rpm;
            motor_ctrl.pwm = u;
            motor_ctrl.duty_value = local_duty;
            xSemaphoreGive(motor_ctrl.mutex);
        }

        setDuty(local_duty);
        // if (need_disable)
        //     disableMotor();
        // UARTprintf("%d, %d,  %d,  %d,  %d\n",
        //            (int)rpm, (int)local_target_rpm, (int)local_duty, (int)avg_acceleration, (int)error);
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
    count++;
    last_hall_update = xTaskGetTickCountFromISR();
    uint32_t tick_delta = last_hall_update - last_tick;
    last_tick = last_hall_update;
    float minute_delta = TICKS_TO_MINUTES(tick_delta);
    if (minute_delta > 0)
        latest_rpm = 1 / (COUNT_PER_REVOLUTION * minute_delta);
    int tmp[3] = {0, 0, 0};
    getHallSensorValues(tmp);
    updateMotor(tmp[0], tmp[1], tmp[2]);
    // xSemaphoreGiveFromISR(xPowerMotorCalcsemaphore, &xMotorTaskWoken);
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
    // UARTprintf("Motor task started\n");
    if (motor_ctrl.mutex == NULL)
    {
        // Handle error
        // UARTprintf("Failed to create mutex\n");
    }

    // configure buttons
    // prvConfigureButton();
    if (xSemaphoreTake(motor_ctrl.mutex, portMAX_DELAY) == pdTRUE)
    {
        motor_ctrl.pwm = 50;
        motor_ctrl.target_rpm = 2000;
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
        // UARTprintf("Failed to take mutex\n");
    }
    /* start motor phase cycle */
    // enableMotor();
    /* Kick start the motor */
    // Do an initial read of the hall effect sensor GPIO lines
    /* read hall sensor gpio lines */
    // UARTprintf("Getting hall values\n");
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
        // UARTprintf("Failed to take mutex\n");
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