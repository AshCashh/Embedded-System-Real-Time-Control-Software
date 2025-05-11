#include "motorlib.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"



bool initMotorLib(uint16_t pwm_period)
{
    UARTprintf("initMotorLib: %d\n", pwm_period);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC |
                    PWM_GEN_MODE_DBG_STOP);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0,
                    (SysCtlClockGet())*pwm_period/MICROSECONDS);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);


    return true;
}

bool getHallSensorValues(int32_t* halls)
{
    // UARTprintf("getHallSensorValues\n");
    /*
        read hall values, returns pin number if high and 0 if low,
        format the data to 1 and 0 by shifting
    */
    halls[0] = GPIOPinRead(HALLA) >> 3;  
    halls[1] = GPIOPinRead(HALLB) >> 2;  
    halls[2] = GPIOPinRead(HALLC) >> 2;  
    return true;  
}

void setDuty(uint16_t duty)
{
    // UARTprintf("setDuty: %d\n", duty);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
        (SysCtlClockGet())*duty/MICROSECONDS);
}

void updateMotor(bool Hall_a, bool Hall_b, bool Hall_c)
{
    uint8_t phase;
    switch (BITS_TO_PHASE(Hall_a, Hall_b, Hall_c))
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
    UARTprintf("enableMotor\n");
    GPIOPinWrite(ENA, 0);
}
void enableMotor()
{
    UARTprintf("disableMotor\n");
    GPIOPinWrite(ENA, GPIO_PIN_6);
}
