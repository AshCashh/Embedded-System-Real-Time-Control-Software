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
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

extern SemaphoreHandle_t xIC2MasterSemaphore;
extern SemaphoreHandle_t xI2CMutex;

extern uint32_t g_ui32SysClock;
/*
 * Sets slave address to ui8Addr
 * Puts ui8Reg followed by two data bytes in *data and transfers
 * over i2c
 */
bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{

    // Add this before taking the mutex or setting up the transfer
    int timeout = 10000;
    while (I2CMasterBusBusy(I2C2_BASE) && --timeout > 0)
    {
        SysCtlDelay(10);
    }
    if (timeout == 0)
    {
        UARTprintf("[!] I2C Bus stuck before read\n");
        recoverI2CBusIfStuck();
    }
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    // Load device slave address
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if (xSemaphoreTake(xIC2MasterSemaphore, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
        I2C_Err();
        xSemaphoreGive(xI2CMutex);
        return false;
    }

    // Send Data
    I2CMasterDataPut(I2C2_BASE, data[0]);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    if (xSemaphoreTake(xIC2MasterSemaphore, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
        I2C_Err();
        xSemaphoreGive(xI2CMutex);
        return false;
    }

    I2CMasterDataPut(I2C2_BASE, data[1]);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if (xSemaphoreTake(xIC2MasterSemaphore, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
        I2C_Err();
        xSemaphoreGive(xI2CMutex);
        return false;
    }

    xSemaphoreGive(xI2CMutex);

    return true;
}

/*
 * Sets slave address to ui8Addr
 * Puts ui8Reg followed by two data bytes in *data and transfers
 * over i2c
 */
bool writeI2C_acc(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data, uint8_t len)
{
    if (len == 0)
        return false;
    // Add this before taking the mutex or setting up the transfer
    int timeout = 10000;
    while (I2CMasterBusBusy(I2C2_BASE) && --timeout > 0)
    {
        SysCtlDelay(10);
    }
    if (timeout == 0)
    {
        UARTprintf("[!] I2C Bus stuck before read\n");
        recoverI2CBusIfStuck();
    }
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false); // Write
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if (xSemaphoreTake(xIC2MasterSemaphore, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
        I2C_Err();
        xSemaphoreGive(xI2CMutex);
        return false;
    }

    for (uint8_t i = 0; i < len - 1; i++)
    {
        I2CMasterDataPut(I2C2_BASE, data[i]);
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        if (xSemaphoreTake(xIC2MasterSemaphore, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
            I2C_Err();
            xSemaphoreGive(xI2CMutex);
            return false;
        }
    }
    I2CMasterDataPut(I2C2_BASE, data[len - 1]);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if (xSemaphoreTake(xIC2MasterSemaphore, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
        I2C_Err();
        xSemaphoreGive(xI2CMutex);
        return false;
    }

    xSemaphoreGive(xI2CMutex);
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
    // Add this before taking the mutex or setting up the transfer
    int timeout = 10000;
    while (I2CMasterBusBusy(I2C2_BASE) && --timeout > 0)
    {
        SysCtlDelay(10);
    }
    if (timeout == 0)
    {
        UARTprintf("[!] I2C Bus stuck before read\n");
        recoverI2CBusIfStuck();
    }
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    // Load device slave address and change I2C to write
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    if (xSemaphoreTake(xIC2MasterSemaphore, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
        I2C_Err();
        xSemaphoreGive(xI2CMutex);
        return false;
    }

    // Load device slave address and change I2C to read
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, true);

    // Read two bytes from I2C
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    if (xSemaphoreTake(xIC2MasterSemaphore, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
        I2C_Err();
        xSemaphoreGive(xI2CMutex);
        return false;
    }

    data[0] = I2CMasterDataGet(I2C2_BASE);

    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    if (xSemaphoreTake(xIC2MasterSemaphore, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
        I2C_Err();
        xSemaphoreGive(xI2CMutex);
        return false;
    }

    data[1] = I2CMasterDataGet(I2C2_BASE);
    xSemaphoreGive(xI2CMutex);

    return true;
}

bool readI2C_acc(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data, uint16_t len)
{
    if (len < 1)
        return false;
    // Add this before taking the mutex or setting up the transfer
    int timeout = 10000;
    while (I2CMasterBusBusy(I2C2_BASE) && --timeout > 0)
    {
        SysCtlDelay(10);
    }
    if (timeout == 0)
    {
        UARTprintf("[!] I2C Bus stuck before read\n");
        recoverI2CBusIfStuck();
    }

    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    // Write register address (no stop)
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false); // Write
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    if (xSemaphoreTake(xIC2MasterSemaphore, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
        I2C_Err();
        xSemaphoreGive(xI2CMutex);
        UARTprintf("[!] READ ERROR    Single Send Failed\n");
        return false;
    }

    // Restart in read mode
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, true); // Read

    // First byte (START)
    I2CMasterControl(I2C2_BASE, len == 1 ? I2C_MASTER_CMD_SINGLE_RECEIVE : I2C_MASTER_CMD_BURST_RECEIVE_START);
    if (xSemaphoreTake(xIC2MasterSemaphore, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
        I2C_Err();

        xSemaphoreGive(xI2CMutex);
        UARTprintf("[!] READ ERROR    First Read Failed\n");
        return false;
    }
    data[0] = I2CMasterDataGet(I2C2_BASE);

    // Middle bytes (CONT)
    for (uint16_t i = 1; i < len - 1; i++)
    {
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        if (xSemaphoreTake(xIC2MasterSemaphore, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
            I2C_Err();

            xSemaphoreGive(xI2CMutex);
            UARTprintf("[!] READ ERROR    Cont Read Failed\n");
            return false;
        }

        data[i] = I2CMasterDataGet(I2C2_BASE);
    }

    // Last byte (FINISH)
    if (len > 1)
    {
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        if (xSemaphoreTake(xIC2MasterSemaphore, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
            I2C_Err();
            xSemaphoreGive(xI2CMutex);
            UARTprintf("[!] READ ERROR     Last Read Failed\n");
            return false;
        }
        data[len - 1] = I2CMasterDataGet(I2C2_BASE);
    }
    xSemaphoreGive(xI2CMutex);

    return true;
}

void I2C_Err(void)
{
    uint32_t err = I2CMasterErr(I2C2_BASE);
    UARTprintf("[!] I2C Read Error: 0x%02X\n", err);

    if (err & I2C_MASTER_ERR_ADDR_ACK)
    {
        UARTprintf("[!] No ACK on address\n");
    }
    if (err & I2C_MASTER_ERR_DATA_ACK)
    {
        UARTprintf("[!] No ACK on data\n");
    }
    if (err & I2C_MASTER_ERR_ARB_LOST)
    {
        UARTprintf("[!] Arbitration lost\n");
    }
    recoverI2CBusIfStuck();
}

void recoverI2CBusIfStuck(void)
{
    if (I2CMasterBusBusy(I2C2_BASE))
    {
        UARTprintf("[!] I2C Bus Busy -- Attempting Recovery\n");
        unstickI2CBus();
        i2cGeneralCallReset(); // maybe working idk
    }
}

bool i2cGeneralCallReset(void)
{
    uint8_t reset_cmd = 0x06; // General Call Software Reset
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);

    I2CMasterSlaveAddrSet(I2C2_BASE, 0x00, false); // General Call
    I2CMasterDataPut(I2C2_BASE, reset_cmd);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    if (xSemaphoreTake(xIC2MasterSemaphore, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        UARTprintf("[!] General Call Reset Failed\n");
        xSemaphoreGive(xI2CMutex);
        return false;
    }

    xSemaphoreGive(xI2CMutex);
    UARTprintf("[*] General Call Reset Sent\n");
    return true;
}

void unstickI2CBus(void)
{
    // Disable I2C peripheral
    I2CMasterDisable(I2C2_BASE);
    SysCtlDelay(1000);

    // Reconfigure SDA and SCL as GPIO outputs
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Both lines high initially (simulate idle state)
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_4 | GPIO_PIN_5);
    SysCtlDelay(1000);

    // Clock SCL 9 times
    for (int i = 0; i < 9; i++)
    {
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, 0); // SCL low
        SysCtlDelay(1000);
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, GPIO_PIN_5); // SCL high
        SysCtlDelay(1000);
    }

    // Simulate STOP condition: SDA high while SCL high
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0); // SDA low
    SysCtlDelay(1000);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, GPIO_PIN_5); // SCL high
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, GPIO_PIN_4); // SDA high
    SysCtlDelay(1000);

    // Return SCL/SDA to I2C mode
    GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);

    // Re-enable I2C
    I2CMasterEnable(I2C2_BASE);
    SysCtlDelay(1000);
    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);
}