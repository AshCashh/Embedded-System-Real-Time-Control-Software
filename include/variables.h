#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "math.h"
/* Defines for motor ctrl */

#define INHA GPIO_PORTF_BASE, GPIO_PIN_1
#define INLA GPIO_PORTF_BASE, GPIO_PIN_2
#define INHB GPIO_PORTF_BASE, GPIO_PIN_3
#define INLB GPIO_PORTG_BASE, GPIO_PIN_0
#define INHC GPIO_PORTL_BASE, GPIO_PIN_4
#define INLC GPIO_PORTL_BASE, GPIO_PIN_5
#define MODE GPIO_PORTL_BASE, GPIO_PIN_0

#define HALLA GPIO_PORTM_BASE, GPIO_PIN_3
#define HALLB GPIO_PORTH_BASE, GPIO_PIN_2
#define HALLC GPIO_PORTN_BASE, GPIO_PIN_2

#define ENA GPIO_PORTC_BASE, GPIO_PIN_6

#define ISENCE_A GPIO_PORTA_BASE, GPIO_PIN_6
#define ISENCE_B GPIO_PORTD_BASE, GPIO_PIN_7
#define ISENCE_C GPIO_PORTE_BASE, GPIO_PIN_3

/* 
    Commutation phases for 3 phase BLDC with INHC = 0
    Phase F loops back to phase A per revolution 
*/
#define PHASE_STOP      0b000
#define PHASE_ALIGN     0b111
#define PHASE_1         0b110                   /* B->C */
#define PHASE_2         0b010                   /* A->C */
#define PHASE_3         0b011                   /* A->B */
#define PHASE_4         0b001                   /* C->B */
#define PHASE_5         0b101                   /* C->A */
#define PHASE_6         0b100                   /* B->A */

/*
    Mathematical constants used to convert between seconds and microseconds for example
    t (microseconds) = t(seconds) / MICROSECONDS
*/
#define MICROSECONDS(duty) \
    (SysCtlClockGet()*duty/1000000)
/* PWM to DUTY*/

#define PWM_TO_DUTY(period_value, pwm) \
    ((pwm*period_value)/100) /* return duty Value */

// #define PWM_FREQUENCY 100000
#define STALL_DURATION 1 //seconds

#define TICKS_TO_MINUTES(t) ( ((float)(t) / (float)configTICK_RATE_HZ) / 60.0f )


#define PID_FREQUENCY 120   //Hz

#define STALL_VAL STALL_DURATION*PID_FREQUENCY

#define COUNT_PER_REVOLUTION 24

#define SECONDS_PER_MINUTE 60

#define ACCELERATION_TOLERANCE 0.95f
#define MAX_ACCELERATION_RPMS (ACCELERATION_TOLERANCE * 500.0f) 
#define MAX_DECELERATION_RPMS (ACCELERATION_TOLERANCE * 500.0f)
#define ESTOP_DECELERATION_RPMS (ACCELERATION_TOLERANCE * 1000.0f)

static inline float count_to_rpm(int count)
{
    return ((float)count / COUNT_PER_REVOLUTION) * PID_FREQUENCY * SECONDS_PER_MINUTE;
}
#define MOVING_AVERAGE_SAMPLES 60
/* PID variables */
#define Kp 0.4f
#define Ki 2.2f 
#define Kd 0.3f

#define dt 1/PID_FREQUENCY 
#define EPSILON 1e-6f

static inline float clamp(float value, float min, float max)
{
    return fminf(fmaxf(value, min), max);
}

/* 
    Commutation phases for 3 phase BLDC with INHC = 1
    Phase F loops back to phase A per revolution 
    Derived motor phase diagram in reference document
*/
// #define PHASE_1            0b001
// #define PHASE_2            0b101
// #define PHASE_3            0b100
// #define PHASE_4            0b110
// #define PHASE_5            0b010
// #define PHASE_6            0b011

/* preprocessor inline convert 3 bit value to individual bits */
#define PHASE_TO_BITS(phase) \
    ((phase & 0b100) >> 2), \
    ((phase & 0b010) >> 1), \
    (phase & 0b001)
#define BITS_TO_PHASE(a, b, c) \
    ((a << 2) | (b << 1) | c)
#define GET_PORT(base, pin)\
    ((base))
#define GET_PIN(base, pin)\
    ((pin))
#define MICROSECONDS_TO_RPM(time_delta_ms)\
    ((1000*60)/(time_delta_ms*6)) /* 6 steps per revolution */
/*Current measure Variales*/

#define VREF 3.3f /* Reference voltage for ADC */
#define VREF_DIV2 1.65f /* Reference voltage divided by 2 */
#define RSHUNT 0.007f /* Shunt resistor value in ohms */
#define GAIN 10.0f /* Gain of the current sense amplifier with a 47K resistor */

#define ADC_MAX_VALUE 4095.0f /* Maximum ADC value for 12-bit resolution */

#define ADC_CURRENT_FREQ 200

#define ADC_CURRENT_SAMPLES 10 /* Number of samples to average for current measurement */

#define Motor_INEFFICIENCY 0.6f

#define MOTOR_NORMAL_VOLTAGE 24.0f /* Normal operating voltage of the motor in volts */

#define AMPS_TO_MILLIAMPS(amps) \
    ((amps) * 1000.0f) /* Convert amps to milliamps for display */



/* Data types for motor configuration and control */
/*
    * @brief Motor control data struct
*/
typedef struct
{
    SemaphoreHandle_t mutex; /* mutex for controlling access */
    float pwm; /* PWM percentage 0-100 */
    volatile uint16_t duty_value; /* current duty cycle value */
    uint16_t period_value; /* current period value */
    uint32_t hall_currents[3]; /* hall sensor currents */
    bool motor_enabled; /* stall prevention flag */
    bool brake; /* brake flag */
    bool Estop; /* emergency stop flag */
    uint8_t stall_counter; /* reactivation count */
    float rpm; /* current rpm value */
    float target_rpm; /* target rpm value */
    int32_t acceleration; /* current acceleration value */
    uint32_t hall_sensor_values[3]; /* hall sensor values */
    /* timestamp */
    uint32_t timestamp; /* timestamp for hall sensor */
} motorcontrol_t;

