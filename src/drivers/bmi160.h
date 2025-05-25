/* header file for BMI160 */

bool sensorBMI160Init(void);
bool sensorBMI160Test(void);
bool sensorBMI160Read(uint8_t *rawData);
static void prvBMI160DataReady(void);