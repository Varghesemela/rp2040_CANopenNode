# Proximity sensor

## Prerequisites

### Files and libraries
- [I2C_comm.cpp](../lib/I2C_comm.cpp)
- [vcnl4040.cpp](../lib/vcnl4040.cpp)
- [config.h](../config.h)
- [main.cpp](../main.cpp)
- hardware gpio library
- hardware i2c library

### Variables
- boolean: 
- [integers](../main.cpp#L98): proxValue, ambientValue, whiteValue
- [constants](../config.h#L46): I2C_Baudrate
- structures: i2c0, i2c1 

## Functions
1. [i2c_begin(i2c_inst_t *i2c, uint Baudrate, uint scl_pin, uint sda_pin)](../lib/I2C_comm.cpp#L14)
2. [i2c_bus_hang_resolve(i2c_inst_t *i2c, uint scl_pin, uint sda_pin)](../lib/I2C_comm.cpp#L40)
3. [isConfigured(i2c_inst_t *i2c)](../lib/I2C_comm.cpp#L62)]
4. [sensorConfiguration(i2c_inst_t *i2c)](../lib/I2C_comm.cpp#L71)]
5. [reg_write(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes)](../lib/vcnl4040.cpp#L31)
6. [reg_read(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes)](../lib/I2C_comm.cpp#L58)
7. [readCommandUpper(i2c_inst_t *i2c, uint8_t commandCode)](../lib/I2C_comm.cpp#L412)
8. [setLEDCurrent(i2c_inst_t *i2c, uint8_t currentValue)](../lib/I2C_comm.cpp#L248)
9. [setIRDutyCycle(i2c_inst_t *i2c, uint16_t dutyValue)](../lib/I2C_comm.cpp#L85)
10. [setProxIntegrationTime(i2c_inst_t *i2c, uint8_t timeValue)](../lib/I2C_comm.cpp#L150)
11. [enableSmartPersistance(i2c_inst_t *i2c)](../lib/I2C_comm.cpp#L192)


## Code routine
1. The two i2c peripherals - i2c0 and i2c1, is initialised for both the sensors by **i2c_begin** function. A handshaking message to the sensor address is sent to confirm comm. establishment.
2. The user/master can request current sensor data by sending '9' command.
3. When the command is received,
    - **is_configured()** is executed to check whether sensors are configured. If not, the sensors are reconfigured using **sensorConfiguration()**.
    - After this, **getProximity()** is executed to obtain the proximity values from the sensor. If the return value is -1, then it is possible that the I2C bus has hanged due to lossy communication. The **i2c_bus_hang_resolve()** is run to clear the bus and reinitiate I2C communication.
4. The user/master can also send 'x' command to restart I2C communication manually.