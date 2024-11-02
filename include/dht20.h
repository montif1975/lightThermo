/* DHT20 TEMPERATURE SENSOR VIA I2C */
#include "hardware/i2c.h"

#define I2C_PORT_SENS               i2c1
#define I2C_SDA_SENS                18
#define I2C_SCL_SENS                19

#define DHT20_I2C_ADDRESS           0x38
#define DHT20_STATUS_OK             0x18
#define DHT20_READ                  0x01
#define DHT20_WRITE                 0x00
#define DHT20_START_MEASURE         0xAC

#define DHT20_WAIT_MEAS_MS          80
#define DTH20_WAIT_MEAS_LOOP        5

#define TEMP_FORMAT_CELSIUS         1
#define TEMP_FORMAT_FAHRENHEIT      2

int DHT20_init(void);
int DHT20_read_data(float *temp, float *hum);

