#ifndef	__I2CDriver__H 
#define __I2CDriver__H

#include "stm32f4xx.h"
class I2CDriver
{
public:
	I2CDriver(){}
	void initialize();
	void setClockSpeed(uint32_t clock_size)
	{
		_i2c.I2C_ClockSpeed = 40000;
	}
	void setSpeed();
	void setMode(uint16_t which_mode)
	{
		_i2c.I2C_Mode = I2C_Mode_I2C;
	}
	void setDutyCycle(uint16_t which_duty_mode)
	{
		_i2c.I2C_DutyCycle = I2C_DutyCycle_16_9 ;
	}
	void setOwnAddress(uint16_t own_address)
	{
		_i2c.I2C_OwnAddress1 = 0x01;
	}
	int writeArrary(uint8_t w_addr, const uint8_t* p_data, uint16_t len);
	int writeByte(uint8_t w_addr, const uint8_t* p_data);
	int readByte(uint8_t r_addr, uint8_t* p_data, uint16_t len);
	int readArrary();
	int cleanReadBuf();
	void setFastMode();
	void setStandardMode();
	~I2CDriver(){}
private:
	void releaseBusByForce();
	I2C_InitTypeDef _i2c;
	GPIO_InitTypeDef _gpio;
		
};

#endif