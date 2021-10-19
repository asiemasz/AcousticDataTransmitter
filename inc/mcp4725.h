#ifndef MCP4725_H
#define MCP4725_H

#define MCP4725_RES 12
#define MCP4725_REF_VOLT 5
#define MCP4725_STEPS 4096
#define MCP4725_MAX_VAL (MCP4725_STEPS - 1)

#include "i2c.h"

typedef enum {
    MCP4725_ADDR_0 = 0x60,
    MCP4725_ADDR_1 =  0x61
} MCP4725_ADDRESS;

typedef enum {
    MCP4725_FAST_MODE =       0x00,
    MCP4725_DAC_MODE =        0x40,
    MCP4725_DAC_EEPROM_MODE = 0x60
} MCP4725_COMMAND_MODE;

typedef enum {
    MCP4725_POWER_DOWN_OFF =    0x00,
    MCP4725_POWER_DOWN_1K =     0x01,
    MCP4725_POWER_DOWN_100K =   0x02,
    MCP4725_POWER_DOWN_500K =   0x03,
} MCP4725_POWER_DOWN_MODE;

typedef enum {
    MCP4725_READ_SETTINGS = 0x1,
    MCP4725_READ_DAC =      0x3,
    MCP4725_READ_EEPROM =   0x5
} MCP4725_READ_MODE;

#define MCP4725_GENERAL_CALL_ADDRESS  0x00
#define MCP4725_GENERAL_CALL_RESET 		0x06
#define MCP4725_GENERAL_CALL_WAKEUP 	0x09

typedef struct MCP4725 {
	struct i2c_device i2c;
	MCP4725_ADDRESS address;
	float refVoltage;
	uint16_t bitsPerVolt;
  MCP4725_POWER_DOWN_MODE powerDownMode;
} MCP4725;

MCP4725 MCP4725_init(I2C_TypeDef * i2c_dev, MCP4725_ADDRESS addr, float vRef, MCP4725_POWER_DOWN_MODE mode);
void MCP4725_setValue(MCP4725* dev, uint16_t value, MCP4725_COMMAND_MODE mode);
void MCP4725_setVoltage(MCP4725* dev, float voltage, MCP4725_COMMAND_MODE mode);
uint16_t MCP4725_getValue(MCP4725* dev);
float MCP4725_getVoltage(MCP4725* dev);
float MCP4725_getEEPROMVoltage(MCP4725 *dev);
uint8_t MCP4725_getPowerType(MCP4725 *dev);
uint8_t MCP4725_getEEPROMPowerType(MCP4725 *dev);
void	MCP4725_reset(MCP4725* dev);
void	MCP4725_wakeUp(MCP4725* dev);

#endif
