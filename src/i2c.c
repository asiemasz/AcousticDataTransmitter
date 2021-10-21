#include "../inc/i2c.h"



void i2c_init(I2C_TypeDef * i2c) {
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	//enable selected i2c peripheral clock
	RCC->APB1ENR |= (1 << ((uint32_t)i2c - APB1PERIPH_BASE)/0x400UL);

	//configure appropriate GPIO pins to be used as SCL, SDA outputs
	struct i2c_pins i2c_pinout;
	if(i2c == I2C1) {
		i2c_pinout = i2c1_pins;
	}
	else if(i2c == I2C2) {
		i2c_pinout = i2c2_pins;
	}
	else {
		i2c_pinout = i2c3_pins;
	}
	//enable peripheral clock for ports of SDA and SCL pins
	gpio_init(i2c_pinout.i2c_scl_port);
	if(i2c_pinout.i2c_scl_port != i2c_pinout.i2c_sda_port)
		gpio_init(i2c_pinout.i2c_sda_port);

	GPIO_pinConfigStruct scl_pin, sda_pin;
	//set SDA and SCL pins as alternate function pins 
	scl_pin.mode = GPIO_MODE_ALTERNATE_FUNCTION;
	sda_pin.mode = GPIO_MODE_ALTERNATE_FUNCTION;

	//set gpio pins output type to open drain
	scl_pin.outType = GPIO_OUT_TYPE_OPEN_DRAIN;
	sda_pin.outType = GPIO_OUT_TYPE_OPEN_DRAIN;

	//set gpio pins output speed to highest possible
	scl_pin.outSpeed = GPIO_OUT_SPEED_VERY_HIGH;
	sda_pin.outSpeed = GPIO_OUT_SPEED_VERY_HIGH;

	//set pull up for both pins
	scl_pin.pullConfiguration = GPIO_PULL_UP;
	sda_pin.pullConfiguration = GPIO_PULL_UP;

	//select appropriate alternate function for both pins
	scl_pin.alternateFunction = i2c_pinout.i2c_scl_af;
	sda_pin.alternateFunction = i2c_pinout.i2c_sda_af;

	gpio_setPinConfiguration(i2c_pinout.i2c_scl_port, i2c_pinout.i2c_scl_pin, &scl_pin);
	gpio_setPinConfiguration(i2c_pinout.i2c_sda_port, i2c_pinout.i2c_sda_pin, &sda_pin);
	
	//reset i2c 
	i2c->CR1 |= I2C_CR1_SWRST_Msk;
	i2c->CR1 &= ~I2C_CR1_SWRST_Msk;

	//set i2c clk frequency (fast)
	i2c->CR2 |= I2C_CR2_FREQ_Msk & (0x2A << I2C_CR2_FREQ_Pos);
	i2c->CCR = (0x23 << I2C_CCR_CCR_Pos) | I2C_CCR_FS;
	i2c->TRISE = 0xd;
	//enable the i2c peripheral
	i2c->CR1 |= I2C_CR1_PE_Msk;
}

static void i2c_start(I2C_TypeDef * i2c) {

	//enable the ACK bit
	//i2c->CR1 |= I2C_CR1_ACK;
	//generate start condition
	i2c->CR1 |= I2C_CR1_START;
	while (!(i2c->SR1 & I2C_SR1_SB)); 		// Wait for EV5

}

static void i2c_stop(I2C_TypeDef * i2c) {
	i2c->CR1 |= I2C_CR1_STOP;
}

static void i2c_byte_write(I2C_TypeDef * i2c, uint8_t data) {
	while(!(i2c->SR1 & I2C_SR1_TXE)); //wait for TXE bit to be set (Data register empty)
	i2c->DR = data;
	while(!(i2c->SR1 & I2C_SR1_BTF));//wait for BTF bit to be set (Byte transfer finished);

}
static void i2c_address_select(I2C_TypeDef *i2c, uint8_t address) {
	i2c->DR = address;
	while (!(i2c->SR1 & I2C_SR1_ADDR));  // wait for ADDR bit to be set
	(void) i2c->SR2;  // read SR1 and SR2 to clear the ADDR bit
}

static void i2c_read_data(I2C_TypeDef* i2c,uint8_t addr, uint8_t* data, uint8_t size) {
	uint8_t remaining = size;
	if(size == 1) {
		//select register address
		i2c_address_select(i2c, addr);
		//clear the ACK bit
		i2c->CR1 &= ~I2C_CR1_ACK;
		//stop i2c
		i2c_stop(i2c);
		//wait for RxNE to be set
		while(!(i2c->SR1 & I2C_SR1_RXNE));
		//read the data from the data register
		data[size - remaining] = (uint8_t) i2c->DR;
	}
	else {
		i2c_address_select(i2c, addr);
		while(remaining > 2) {
			i2c->CR1 |= I2C_CR1_ACK; 

			//wait for RxNE bit to be set
			while(!(i2c->SR1 & I2C_SR1_RXNE));
			//read data from data register
			data[size - remaining] = (uint8_t) i2c->DR;
			//set the ACK bit to ackonwledge the data received
			--remaining;
		}
		//read the second last byte
		//wait for RxNE bit to be set
		i2c->CR1 |= I2C_CR1_ACK; 
		while(!(i2c->SR1 & I2C_SR1_RXNE));
		data[size - remaining] =(uint8_t) i2c->DR;
		//clear the ACK bit
		i2c->CR1 &= ~I2C_CR1_ACK;
		i2c_stop(i2c);
		remaining--;
		//read the last byte
		while(!(i2c->SR1 & I2C_SR1_RXNE));
		data[size - remaining] =(uint8_t) i2c->DR;
	}
}

void i2c_write(i2c_device* dev, uint8_t reg, uint8_t data) {
	i2c_start(dev->i2c);
	i2c_address_select(dev->i2c,dev->device_addr);
	i2c_byte_write(dev->i2c,reg);
	i2c_byte_write(dev->i2c,data);
	i2c_stop(dev->i2c);
}

void i2c_read(i2c_device* dev, uint8_t reg, void* data, uint8_t size) {
	i2c_start(dev->i2c);
	i2c_address_select(dev->i2c,dev->device_addr);
	i2c_byte_write(dev->i2c,reg);
	i2c_stop(dev->i2c);
	i2c_start(dev->i2c);
	i2c_read_data(dev->i2c, dev->device_addr+0x1, data, size);
	i2c_stop(dev->i2c);
}

void i2c_transmit(i2c_device* dev, uint8_t* data, uint8_t length) {
	i2c_start(dev->i2c);
	i2c_address_select(dev->i2c, dev->device_addr);
	for(int i = 0; i < length; i++) {
		i2c_byte_write(dev->i2c, data[i]);
	}
	i2c_stop(dev->i2c);
}

void i2c_receive(i2c_device* dev, uint8_t* data, uint8_t length) {
	uint8_t remaining = length;

	i2c_start(dev->i2c);

	//set the ACK bit to ackonwledge the data received
	dev->i2c->CR1 |= I2C_CR1_ACK; 
	
	i2c_address_select(dev->i2c, dev->device_addr+0x1);
	
	while(remaining > 2) {
		dev->i2c->CR1 |= I2C_CR1_ACK; 

		//wait for RxNE bit to be set
		while(!(dev->i2c->SR1 & I2C_SR1_RXNE));
		//read data from data register
		data[length - remaining] = (uint8_t) dev->i2c->DR;

		--remaining;
	}
	//read the second last byte
	//wait for RxNE bit to be set
	dev->i2c->CR1 |= I2C_CR1_ACK; 
	while(!(dev->i2c->SR1 & I2C_SR1_RXNE));
	data[length - remaining] = (uint8_t) dev->i2c->DR;
		//clear the ACK bit
	dev->i2c->CR1 &= ~I2C_CR1_ACK;
	i2c_stop(dev->i2c);
	remaining--;
	//read the last byte
	while(!(dev->i2c->SR1 & I2C_SR1_RXNE));
	data[length - remaining] =(uint8_t) dev->i2c->DR;
}


