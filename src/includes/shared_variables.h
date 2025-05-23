#include <stdint.h>

volatile char g_pcMotorStatus[16] = "OFF"; // Shared motor status
volatile uint32_t g_ui32MotorRPM = 0;      // Shared motor RPM

