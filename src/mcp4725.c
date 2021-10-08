#include "../inc/mcp4725.h"

static void MCP4725_writeCommand(MCP4725* dev, uint16_t value, MCP4725_COMMAND_MODE mode, MCP4725_POWER_DOWN_MODE power) {
	uint8_t buffer[3];
	switch(mode) {
		case MCP4725_FAST_MODE:
			buffer[0] = mode | (power << 4) | ((uint8_t)(value/256));
			buffer[1] = ((uint8_t) (value % 256));

			i2c_transmit(&(dev->i2c), buffer, 2);
			break;

		case MCP4725_DAC_MODE:
		case MCP4725_DAC_EEPROM_MODE:
			buffer[0] = mode | (power << 1);
			buffer[1] = (uint8_t)(value/256);
			buffer[2] = (uint8_t)(value%256);

			i2c_transmit(&(dev->i2c), buffer, 3);
			break;
	}
}

static uint16_t MCP4725_readData(MCP4725* dev, MCP4725_READ_MODE mode) {
	uint8_t buffer[mode];
	uint16_t ret;
	
	i2c_receive(&dev->i2c, buffer, mode);
	switch(mode) {
		case MCP4725_READ_SETTINGS:
			ret = buffer[0];
			break;
		case MCP4725_READ_DAC:
		case MCP4725_READ_EEPROM:
			ret = buffer[mode-2];
			ret = (ret << 8) | buffer[mode-1];
			break;
	}

	return ret;

}

MCP4725 MCP4725_init(I2C_TypeDef * i2c_dev, MCP4725_ADDRESS addr, float vRef, MCP4725_POWER_DOWN_MODE mode) {
	MCP4725 _dev;
	_dev.address = (uint8_t)(addr << 1);
	_dev.i2c.i2c = i2c_dev;
	_dev.i2c.device_addr = _dev.address;
	if(i2c_dev == I2C1)
		_dev.i2c.pins = i2c1_pins;
	else if(i2c_dev == I2C2)
		_dev.i2c.pins = i2c2_pins;
	else 
		_dev.i2c.pins = i2c3_pins;
	if(vRef == 0)
		_dev.refVoltage = MCP4725_REF_VOLT;
	else
		_dev.refVoltage = vRef;
	_dev.bitsPerVolt = (float)MCP4725_STEPS / _dev.refVoltage;
	if(mode == 0)
		_dev.powerDownMode = MCP4725_POWER_DOWN_OFF;
	else
		_dev.powerDownMode = mode;
	return _dev;
} 

void MC4725_setValue(MCP4725* dev, uint16_t value, MCP4725_COMMAND_MODE mode) {
	MCP4725_writeCommand(dev, value, mode, dev->powerDownMode);
}

void MCP4725_setVoltage(MCP4725* dev, float voltage, MCP4725_COMMAND_MODE mode) {
	uint16_t value = 0;
	if (voltage <= 0) {
		value = MCP4725_MAX_VAL;
	}
	else {
		value = voltage * dev->bitsPerVolt;
	}
	MCP4725_writeCommand(dev, value, mode, dev->powerDownMode);
}

uint16_t MCP4725_getValue(MCP4725* dev) {
	uint16_t value = MCP4725_readData(dev, MCP4725_READ_DAC);
	return value >> 4;
}

float MCP4725_getVoltage(MCP4725* dev) {
	float value = (float) MCP4725_getValue(dev);
	return value / (float) dev->bitsPerVolt;
}

float MCP4725_getEEPROMVoltage(MCP4725 *dev) {
	uint16_t value = MCP4725_readData(dev, MCP4725_READ_EEPROM);
	value = value & 0x0FFF;
	return ((float)value)/((float) dev->bitsPerVolt);
}

uint8_t MCP4725_getPowerType(MCP4725 *dev) {
	uint8_t value = MCP4725_readData(dev, MCP4725_READ_SETTINGS);

	value = value & 0x06;
	return (value >> 1);
}

uint8_t MCP4725_getEEPROMPowerType(MCP4725 *dev) {
	uint16_t value = MCP4725_readData(dev, MCP4725_READ_EEPROM);
	value = value << 1;
	return (uint8_t) (value >> 14);
}

void MCP4725_reset(MCP4725* dev) {
	uint8_t buffer[1] = {MCP4725_GENERAL_CALL_RESET};
	i2c_device _dev;
	_dev.device_addr = MCP4725_GENERAL_CALL_ADDRESS;
	_dev.i2c = dev->i2c.i2c;
	_dev.pins = dev->i2c.pins;
	i2c_transmit(&_dev, buffer, 1);
}

void	MCP4725_wakeUp(MCP4725* dev) {
	uint8_t buffer[1] = {MCP4725_GENERAL_CALL_WAKEUP};
	i2c_device _dev;
	_dev.device_addr = MCP4725_GENERAL_CALL_ADDRESS;
	_dev.i2c = dev->i2c.i2c;
	_dev.pins = dev->i2c.pins;
	i2c_transmit(&_dev, buffer, 1);
}

