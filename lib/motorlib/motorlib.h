/*
 * motorlib.h
 *
 *  Created on: May 2025
 *      Author: EGH456
 */

#ifndef MOTORLIB_H_
#define MOTORLIB_H_

#ifdef __cplusplus
extern "C" {
#endif

// #include <xdc/runtime/Error.h>
#include <stdbool.h>
#include <stdint.h>

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
#define MICROSECONDS 1000000





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


/*!
    *  @brief  measures hall sensor lines to get current phase / values
    *
    *  @param halls array of hall sensor values
    * 
    *  @pre    motor library has been initialized using initMotorLib() and duty cycle as been set
    * 
    *  @returns hall_a, hall_b, hall_c
*/
bool getHallSensorValues(int32_t* halls);

#define PWM_FREQUENCY 100000

/*!
 *  @brief  This function sets the duty cycle of the high side PWM lines.
 *
 *  @pre    motor library has been initialized using initMotorLib()
 *
 *  @param  duty  16bit integer number for the setting the duty cycle
 *                in microseconds where the max integer = PWM Period
 *                set prior in the initMotorLib() function
 *                Valid values for duty are 0 - PWMStruct.MaxDuty
 *
 */
void setDuty(uint16_t duty);

/*!
 *  @brief  Main function which Commutates the motor phases A,B,C to the correct values based on the Hall sensor input.
 *
 *  @pre    motor library has been initialized using initMotorLib() and duty cycle as been set
 *
 *  @param  Hall_a         current value of the Hall A effect sensor as a bool (0 or 1)
 *
 *  @param  Hall_b         current value of the Hall B effect sensor as a bool (0 or 1)
 *
 *  @param  Hall_c         current value of the Hall C effect sensor as a bool (0 or 1)
 * *
 */
void updateMotor(bool Hall_a, bool Hall_b, bool Hall_c);


/*!
 *  @brief  Brakes motor by turning all phases high or low.
 *
 *  @pre    motor library has been initialized using initMotorLib() and duty cycle as been set
 *
 *  @param  brakeType      Determines hard or soft brake. If true then all phases set to high, else all phases set to low
 *
 * *
 */
void stopMotor(bool brakeType);

/*!
 *  @brief  Enables Motor Drive by setting enable pin to low
 *
 *
 * *
 */
void enableMotor();

/*!
 *  @brief  Disables Motor Drive by setting enable pin to High
 *
 *
 * *
 */
void disableMotor();

/*!
 *  @brief  Initialise GPIO and PWM module to ensure correct setup of High/Low side pins.
 *
 *  @param  pwm_duty_period   16 bit unsigned integer pwm period is in microseconds and should be set to be a value between 10 - 100 (100KHz - 10Khz)
 *
 *  @return true if the PWM modules initialised successfully. False if an error occurred likely due to already being opened elsewhere
 *
 * *
 */
bool initMotorLib(uint16_t pwm_period);

/*!
 *  @brief  getter function for internal PWM period.
 *
 *
 *  @param  pwm_period      recommended value is 24, relates to pwm frequency
 *                          and determines resolution of duty cycle value
 *                          see PWM device information to understand how pwm period limits range of duty cycle
 *                          max duty becomes pwm_period value
 *
 *
 *  @return bool            if initialisation was successful, error_block can be used to determine further issues
 *                          on what caused a failure to setup the motor.
 *
 * *
 */
uint16_t getMotorPWMPeriod();

#ifdef __cplusplus
}
#endif

#endif /* MOTORLIB_H_ */


