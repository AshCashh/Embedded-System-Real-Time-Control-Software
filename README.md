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

---

**Course:** EGH456 Embedded Systems  
**University:** Queensland University of Technology (QUT)
