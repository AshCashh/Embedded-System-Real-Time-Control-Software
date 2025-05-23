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

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

/* Slave address */
#define BMI160_IC2_ADDRESS_GND 0x68   // address if SDO pin is pulled to GND
#define BMI160_IC2_ADDRESS_VDDIO 0x69 // address if SDO pin is pulled to VDDIO

/* Register addresses */
// #define REG_RESULT                      0x00
// #define REG_CONFIGURATION               0x01
// #define REG_LOW_LIMIT                   0x02
// #define REG_HIGH_LIMIT                  0x03

#define MAG_GYR_ACC 0x04 // reigster address for if we want mag aswel
#define REG_ACC_RESULT 0x12     // register adress for if we just want gyro and acceleration
#define REG_CHIP_ID 0x00
#define REG_ERROR 0x02
#define REG_CMD 0x7E // Command register triggers operations like softreset, NVM programming etc
#define REG_ACC_CONF 0x40 // acceleration sensor config register
#define REG_ACC_RANGE 0x41 // selection of the accelerometer g-range.




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
    // softreset value to cmd reg
    uint8_t val = 0xB6;
    // reset sensor
    if (!writeI2C_acc(BMI160_IC2_ADDRESS_VDDIO, REG_CMD, &val, 1)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // enable accelerometer
    // Repeatedly attempt to set accel to normal mode (it wont wake up sometimes)
    uint8_t pwd = 0x00;
    while (pwd == 0x00) {
        val = 0x11;
        writeI2C_acc(BMI160_IC2_ADDRESS_VDDIO, 0x7E, &val, 1);
        vTaskDelay(pdMS_TO_TICKS(10));

        readI2C(BMI160_IC2_ADDRESS_VDDIO, 0x03, &val); // PMU_STATUS
        if ((val & 0x30) == 0x10) {
            UARTprintf("[W] Accel in Normal Mode\n");
            break;
        }

        UARTprintf("[!] Accel still in suspend (PMU_STATUS: 0x%02X), retrying...\n", val);
    }


    uint8_t acc_range = 0b0011;
    if (!writeI2C_acc(BMI160_IC2_ADDRESS_VDDIO, REG_ACC_RANGE, &acc_range, 1)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    // config accelerometer
    // This is actually done via ACC_CONF (0x40) and ACC_RANGE (0x41)
    val = 0x28; // 0x08 << 4 | 0x03 -> ODR 100Hz, ±2g
    if (!writeI2C(BMI160_IC2_ADDRESS_VDDIO, REG_ACC_CONF, &val)) {
        return false;
    }

    uint8_t val1;
    readI2C(BMI160_IC2_ADDRESS_VDDIO, 0x40, &val1);
    UARTprintf("ACC_CONF = 0x%02X\n", val1);  // Expect 0x28
    readI2C(BMI160_IC2_ADDRESS_VDDIO, 0x41, &val1);
    UARTprintf("ACC_RANGE = 0x%02X\n", val1); // Expect 0x03
    uint8_t pmu;
    readI2C(BMI160_IC2_ADDRESS_VDDIO, 0x03, &pmu);
    UARTprintf("PMU_STATUS = 0x%02X\n", pmu);

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
    uint8_t val;

    UARTprintf("FINDING CHIP ID:\n");
    // Check manufacturer ID
    readI2C(BMI160_IC2_ADDRESS_VDDIO, REG_CHIP_ID, (uint8_t *)&val);

    if (val != CHIP_ID)
    {
        return false;
    }

    // print Man ID
    UARTprintf("CHIP ID Correct: %d\n", val);

    return (true);
}

// /**************************************************************************************************
//  * @fn          sensorOpt3001Convert
//  *
//  * @brief       Convert raw data to object and ambience temperature
//  *
//  * @param       rawData - raw data from sensor
//  *
//  * @param       convertedLux - converted value (lux)
//  *
//  * @return      none
//  **************************************************************************************************/
// void sensorOpt3001Convert(uint16_t rawData, float *convertedLux)
// {
// 	uint16_t e, m;

// 	m = rawData & 0x0FFF;
// 	e = (rawData & 0xF000) >> 12;

// 	*convertedLux = m * (0.01 * exp2(e));
// }
