/**************************************************************************************************
*  Filename:       i2cOptDriver.c
*  By:             Jesse Haviland
*  Created:        1 February 2019
*  Revised:        23 March 2019
*  Revision:       2.0
*
*  Description:    i2c Driver for use with opt3001.c and the TI OP3001 Optical Sensor
*************************************************************************************************/

// ----------------------- Includes -----------------------
#include "i2cOptDriver.h"
#include "inc/hw_memmap.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "driver_lib/sysctl.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern SemaphoreHandle_t xIC2MasterSemaphore;
/*
 * Sets slave address to ui8Addr
 * Puts ui8Reg followed by two data bytes in *data and transfers
 * over i2c
 */
bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    // Load device slave address
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if (xSemaphoreTake(xIC2MasterSemaphore, portMAX_DELAY) != pdTRUE)
        return false;
    // Send Data
    I2CMasterDataPut(I2C0_BASE, data[0]);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    if (xSemaphoreTake(xIC2MasterSemaphore, portMAX_DELAY) != pdTRUE)
        return false;
    I2CMasterDataPut(I2C0_BASE, data[1]);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if (xSemaphoreTake(xIC2MasterSemaphore, portMAX_DELAY) != pdTRUE)
        return false;

    return true;
}

/*
 * Sets slave address to ui8Addr
 * Writes ui8Reg over i2c to specify register being read from
 * Reads three bytes from i2c slave. The third is redundant but
 * helps to flush the i2c register
 * Stores first two received bytes into *data
 */
bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    // Load device slave address and change I2C to write
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    if (xSemaphoreTake(xIC2MasterSemaphore, portMAX_DELAY) != pdTRUE)
        return false;
    // Load device slave address and change I2C to read
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, true);

    // Read two bytes from I2C
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    if (xSemaphoreTake(xIC2MasterSemaphore, portMAX_DELAY) != pdTRUE)
        return false;
    data[0] = I2CMasterDataGet(I2C0_BASE);

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    if (xSemaphoreTake(xIC2MasterSemaphore, portMAX_DELAY) != pdTRUE)
        return false;
    data[1] = I2CMasterDataGet(I2C0_BASE);

    return true;
}


