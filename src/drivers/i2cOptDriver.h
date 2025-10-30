/**************************************************************************************************
*  Filename:       i2cOptDriver.h
*  By:             Jesse Haviland
*  Created:        1 February 2019
*  Revised:        2 February 2019
*  Revision:       1.0
*
*  Description:    i2c Driver for use with the Texas Instruments OP3001 Optical Sensor
*************************************************************************************************/



#ifndef _I2COPTDRIVER_H_
#define _I2COPTDRIVER_H_



// ----------------------- Includes -----------------------
#include <stdbool.h>
#include <stdint.h>



// ----------------------- Exported prototypes -----------------------
bool waitForAck(void);
extern bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data);
extern bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data);
extern bool readI2Cmul(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data, uint16_t len);
bool writeI2Cmul(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data, uint8_t len);
// void unstickI2CBus(void);
// bool i2cGeneralCallReset(void);
// void recoverI2CBusIfStuck(void);

#endif /* _I2COPTDRIVER_H_ */
