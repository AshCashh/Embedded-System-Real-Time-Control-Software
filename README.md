# Electric Vehicle Embedded System – Real-Time Control Software

Battery-electric and hybrid-electric vehicles are gaining popularity due to their increasing affordability, fuel efficiency, and environmental benefits. The automotive industry is rapidly transitioning towards electric vehicle (EV) design, with recent advancements in autonomous electric vehicles further accelerating this shift.

As an embedded systems engineer, you have been contracted by an automotive manufacturer transitioning to battery-electric technology. Your task is to **design the real-time sensing, motor control, and user interface software** for the embedded system of an electric vehicle.

## Objectives

The primary goal is to develop embedded software that ensures the **safe monitoring and control** of the electric vehicle, including the sensing and actuation of a **3-phase Brushless DC (BLDC) motor**.

### Your software must support:
- Monitoring the motor’s state (rotational velocity, power consumption, etc.)
- Managing motor start-up, braking, and emergency shutdown procedures
- Handling multiple real-time tasks, such as:
  - Sensor data acquisition and filtering
  - Fault detection and handling
  - Display of critical system information

## Hardware Setup

You will be provided with:
- A **Tiva TM4C1294NCPDT** microcontroller development kit
- A **motor testing kit** including:
  - Motor driver board
  - Sensor boards
- A compatible electrical interface for sensor and motor connections

## Resources

Additional resources and documentation will be provided:
- Microcontroller pin mapping (GPIO, I2C, ADC, UART)
- Motor driver input/output specifications
- Motor operation description
- **MotorLib** – A custom motor driver library
- Motor kit setup guide

## Motor calcs and reference information

### DRV8323 shunt resistor current calculation
$$I_{\text{SOx}} = \frac{\frac{V_{\text{ref}}}{2} - V_{\text{SOx}}}{G_{\text{CSA}} \times R_{\text{SENSE}}}$$
Gain setting set by 47K resistor to 10, resistor is 0.007 $\Omega$, $V_{\text{ref}}$ is 3.3V, $V_{\text{SOx}}$ is being measured
$$I_{\text{SOx}} = \frac{\frac{3.3}{2} - V_{\text{SOx}}}{10 \times 0.007} $$
$$I_{\text{SOx}} = \frac{1.65 - V_{\text{SOx}}}{0.7} $$
---

**Course:** EGH456 Embedded Systems  
**University:** Queensland University of Technology (QUT)

page 31 drv8323 for motors


## Motor todo list 
[] Desired rpm
[] Control loop
[] Within deceleration and acceleration spec
[] E-stop
[] Event queue trigger for conditions
[] Current sensing