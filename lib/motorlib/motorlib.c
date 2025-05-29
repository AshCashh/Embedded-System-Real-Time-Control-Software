#include "motorlib.h"



bool initMotorLib(uint16_t pwm_period)
{
    UARTprintf("initMotorLib: %d\n", pwm_period);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC |
                    PWM_GEN_MODE_DBG_STOP);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0,
        MICROSECONDS(pwm_period));
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);


    return true;
}

uint8_t getHallSensorValues()
{
    // UARTprintf("getHallSensorValues\n");
    /*
        read hall values, returns pin number if high and 0 if low,
        format the data to 1 and 0 by shifting
    */
    int tmp[3] = {0, 0, 0};
    tmp[0] = GPIOPinRead(HALLA) >> 3;  
    tmp[1] = GPIOPinRead(HALLB) >> 2;  
    tmp[2] = GPIOPinRead(HALLC) >> 2;
    uint8_t halls = BITS_TO_PHASE(tmp[0], tmp[1], tmp[2]);

    return halls;  
}

void setDuty(float duty)
{
    // UARTprintf("setDuty: %d\n", duty);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,  (int)(SysCtlClockGet() * (float)(duty / 1000000.0f)));
}

void updateMotor()
{
    uint8_t current_phase = getHallSensorValues();
    uint8_t phase;
    switch (current_phase)
    {
        case PHASE_1:
            // UARTprintf("PHASE_1, Updating to (%d,%d,%d)\n", PHASE_TO_BITS(PHASE_2));
            phase = PHASE_2;
            break;
        case PHASE_2:
            // UARTprintf("PHASE_2, Updating to (%d,%d,%d)\n", PHASE_TO_BITS(PHASE_3));
            phase = PHASE_3;
            break;
        case PHASE_3:
            // UARTprintf("PHASE_3, Updating to (%d,%d,%d)\n", PHASE_TO_BITS(PHASE_4));
            phase = PHASE_4;
            break;
        case PHASE_4:
            // UARTprintf("PHASE_4, Updating to (%d,%d,%d)\n", PHASE_TO_BITS(PHASE_5));
            phase = PHASE_5;
            break;
        case PHASE_5:
            // UARTprintf("PHASE_5, Updating to (%d,%d,%d)\n", PHASE_TO_BITS(PHASE_6));
            phase = PHASE_6;
            break;
        case PHASE_6:
            // UARTprintf("PHASE_6, Updating to (%d,%d,%d)\n", PHASE_TO_BITS(PHASE_1));
            phase = PHASE_1;
            break;
        case PHASE_STOP:
            // UARTprintf("PHASE_STOP\n");
            phase = PHASE_STOP;
            break;
        case PHASE_ALIGN:
            // UARTprintf("PHASE_ALIGN\n");
            phase = PHASE_ALIGN;
            break;
        default:
            break;
    }
    /* write bit 1 to INLA, 2 to INHB, 3 to INLB */
    int hall_a = (phase & 0b100) >> 2 ? GPIO_PIN_2 : 0;
    int hall_b = (phase & 0b010) >> 1 ? GPIO_PIN_3 : 0;
    int hall_c = (phase & 0b001) ? GPIO_PIN_0 : 0;
    GPIOPinWrite(INLA, hall_a);
    GPIOPinWrite(INHB, hall_b);
    GPIOPinWrite(INLB, hall_c);
    // /* print gpio pin write of each hall */
    // UARTprintf("updateMotor: %d %d %d\n",
    //     hall_a, hall_b, hall_c); 
}

void stopMotor(bool brakeType)
{
    // UARTprintf("stopMotor: brake=%d\n", brakeType);
    uint8_t val = brakeType ? 0xFF : 0x00;
    GPIOPinWrite(INLA, val);
    GPIOPinWrite(INHB, val);
    GPIOPinWrite(INLB, val);
    GPIOPinWrite(INLC, val);
}

void disableMotor()
{
    // UARTprintf("disabledMotor\n");
    GPIOPinWrite(ENA, 0);
}
void enableMotor()
{
    // UARTprintf("enabledMotor\n");
    GPIOPinWrite(ENA, GPIO_PIN_6);
}
