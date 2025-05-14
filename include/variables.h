#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
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

#define PWM_FREQUENCY 100000




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

/* Data types for motor configuration and control */
/*
    * @brief Motor control data struct
*/
typedef struct
{
    SemaphoreHandle_t mutex; /* mutex for controlling access */
    volatile uint16_t duty_value; /* current duty cycle value */
    uint16_t period_value; /* current period value */
    uint32_t hall_currents[3]; /* hall sensor currents */
    bool brake; /* brake flag */
    uint32_t rpm; /* current rpm value */
    uint32_t hall_sensor_values[3]; /* hall sensor values */
} motorcontrol_t;

