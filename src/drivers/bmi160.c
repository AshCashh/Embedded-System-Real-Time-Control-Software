/* Control for BMI160 */
/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "i2cOptDriver.h"
#include "bmi160.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "driverlib/sysctl.h"
#include "task.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

extern uint32_t g_ui32SysClock;

/* Slave address */
#define BMI160_IC2_ADDRESS_GND 0x68   // address if SDO pin is pulled to GND
#define BMI160_IC2_ADDRESS_VDDIO 0x69 // address if SDO pin is pulled to VDDIO

/* Register addresses */
// #define REG_RESULT                      0x00
// #define REG_CONFIGURATION               0x01
// #define REG_LOW_LIMIT                   0x02
// #define REG_HIGH_LIMIT                  0x03

#define MAG_GYR_ACC 0x04    // reigster address for if we want mag aswel
#define REG_ACC_RESULT 0x12 // register adress for if we just want gyro and acceleration
#define REG_CHIP_ID 0x00    // chip ID storage location
#define REG_ERROR 0x02
#define REG_CMD 0x7E         // Command register triggers operations like softreset, NVM programming etc
#define REG_ACC_CONF 0x40    // acceleration sensor config register
#define REG_ACC_RANGE 0x41   // selection of the accelerometer g-range.
#define REG_INT_EN 0x50      // enable interrupts register
#define REG_PMU_STATUS 0x03  // read power status of chip
#define REG_IN_OUT_CTRL 0x53 // pullup for INT1 and 2 pins
#define REG_DR_INT_MAP 0x56  // data ready interrupt map to either INT1 (7:4) or INT2 (3:0)

/* Register values */
#define CHIP_ID 0xD1 // chip id check
// #define DEVICE_ID                       0x3001  // Device ID = 3001

// #define CONFIG_RESET                    0xC810
// #define CONFIG_TEST                     0xCC10

#define CONFIG_ENABLE 0x10C4  // sensor receives this as 0xC410 as upper and lower bytes are received in reverse
#define CONFIG_DISABLE 0x10C0 // sensor receives this as 0xC010 as upper and lower bytes are received in reverse

// /* Bit values */
// #define DATA_RDY_BIT                    0x0080  // Data ready

// /* Register length */
// #define REGISTER_LENGTH                 2

// /* Sensor data size */
// #define DATA_LENGTH                     2

/**************************************************************************************************
 * @fn          sensorBMI160Init
 *
 * @brief       Initialize the acceleration sensor by reseting the sensor
 *
 * @return      none
 **************************************************************************************************/
bool sensorBMI160Init(void)
{
    vTaskDelay(pdMS_TO_TICKS(300));
    // softreset value to cmd reg
    uint8_t val = 0xB6;
    // reset sensor
    if (!writeI2C_acc(BMI160_IC2_ADDRESS_VDDIO, REG_CMD, &val, 1))
    {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(150));
    // Disable Advanced Power Save
    uint8_t pwr_conf = 0x00;
    writeI2C_acc(0x69, 0x6C, &pwr_conf, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // enable accelerometer
    val = 0x11;
    writeI2C_acc(BMI160_IC2_ADDRESS_VDDIO, 0x7E, &val, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    readI2C(BMI160_IC2_ADDRESS_VDDIO, REG_PMU_STATUS, &val); // PMU_STATUS
    if (!((val & 0x30) == 0x10))
    {
        UARTprintf("[W] Accel not in Normal Mode\n");
    }

    uint8_t acc_range = 0b0011;
    if (!writeI2C_acc(BMI160_IC2_ADDRESS_VDDIO, REG_ACC_RANGE, &acc_range, 1))
    {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    // config accelerometer
    val = 0x28; // freq
    if (!writeI2C_acc(BMI160_IC2_ADDRESS_VDDIO, REG_ACC_CONF, &val, 1))
    {
        return false;
    }

    /* ------ Set up Interrupts on chip ------ */
    // map data ready interrupt to INT1
    uint8_t int1_en_dr = (1 << 7); // enable data ready interrupt
    if (!writeI2C_acc(BMI160_IC2_ADDRESS_VDDIO, REG_DR_INT_MAP, &int1_en_dr, 1))
    {
        return false;
    }

    // Pull up resistor for INT1
    uint8_t int_behaviour =
        (1 << 3)    // INT1 output enabled
        | (0 << 2)  // INT1 push-pull/open-drain
        | (1 << 1); // INT1 active high
    if (!writeI2C_acc(BMI160_IC2_ADDRESS_VDDIO, REG_IN_OUT_CTRL, &int_behaviour, 1))
    {
        return false;
    }

    // enable data ready interrupt, 1 for [1] mask
    uint8_t data_ready_int = (1 << 4);
    if (!writeI2C_acc(BMI160_IC2_ADDRESS_VDDIO, (REG_INT_EN | 0x01), &data_ready_int, 1))
    { // 0x01 OR for selecting second field
        return false;
    }

    prvBMI160DataReady();

    // uint8_t val1;
    // readI2C(BMI160_IC2_ADDRESS_VDDIO, 0x40, &val1);
    // UARTprintf("ACC_CONF = 0x%02X\n", val1); // Expect 0x28
    // readI2C(BMI160_IC2_ADDRESS_VDDIO, 0x41, &val1);
    // UARTprintf("ACC_RANGE = 0x%02X\n", val1); // Expect 0x03
    // uint8_t pmu;
    // readI2C(BMI160_IC2_ADDRESS_VDDIO, 0x03, &pmu);
    // UARTprintf("PMU_STATUS = 0x%02X\n", pmu);

    return true;
}

/**************************************************************************************************
 * @fn          sensorBMI160Read
 *
 * @brief       Read the result register
 *
 * @param       Buffer to store data in
 *
 * @return      TRUE if valid data
 **************************************************************************************************/
bool sensorBMI160Read(uint8_t *rawData)
{
    if (!readI2C_acc(BMI160_IC2_ADDRESS_VDDIO, REG_ACC_RESULT, rawData, 6))
    {
        return false;
    }

    return true;
}

/**************************************************************************************************
 * @fn          sensorBMI160Test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 **************************************************************************************************/
bool sensorBMI160Test(void)
{
    // UARTprintf("FINDING CHIP ID:\n");
    uint8_t val = 0;
    for (int i = 0; i < 10; i++)
    {
        readI2C(BMI160_IC2_ADDRESS_VDDIO, REG_CHIP_ID, &val);
        if (val == CHIP_ID)
            break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (val != CHIP_ID)
    {
        UARTprintf("[!] Chip ID failed: 0x%02X\n", val);
        return false;
    }

    // UARTprintf("CHIP ID Correct: %d\n", val);
    return true;
}

// config BMI160 data ready interrupt on Port P Pin 3
static void prvBMI160DataReady(void)
{
    // Enable GPIO port for the INT pin
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Configure pull-up resistor?
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // trigger on the falling edge
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_4);
    // enable GPIOP interrupt
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_4);

    IntEnable(INT_GPIOD);
}

#define BMI160_I2C_ADDR  0x69
#define BMI160_ERR_REG   0x02

bool bmi160ReadErrorStatus(uint8_t *err)
{
    if (err == NULL) return false;

    if (!readI2C(BMI160_I2C_ADDR, BMI160_ERR_REG, err))
    {
        UARTprintf("[!] Failed to read BMI160 ERR_REG\n");
        return false;
    }

    UARTprintf("[*] BMI160 ERR_REG = 0x%02X\n", *err);

    // Decode some known bits if needed
    if (*err & 0x01)
        UARTprintf("    [!] Fatal error detected\n");
    if (*err & 0x02)
        UARTprintf("    [!] Error in I2C communication\n");
    if (*err & 0x04)
        UARTprintf("    [!] SPI error\n");
    if (*err & 0x08)
        UARTprintf("    [!] Invalid register write\n");

    return true;
}