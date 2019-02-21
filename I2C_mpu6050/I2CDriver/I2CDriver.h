#ifndef	__I2CDriver__H 
#define __I2CDriver__H

#include <stm32f4xx.h>

#define CLEAR_ADDR_BIT			I2C_ReadRegister(I2C1, I2C_Register_SR1);\
								I2C_ReadRegister(I2C1, I2C_Register_SR2);\

const int I2Cx_FLAG_TIMEOUT =	60;		//test result: max 52 times,sometime is 2 times and 51 times
const int I2Cx_LONG_TIMEOUT = 80;		//give the allowance 
const int I2Cx_RECIEVE_TIMEOUT = 400;	//test result: mostly 282 times
const int I2Cx_SEND_TIMEOUT = 80;

class I2CDriver
{
public:
	I2CDriver()
	{
		_clock_speed = 40000;
		_i2c_mode = I2C_Mode_I2C;
		_duty_cycle = I2C_DutyCycle_16_9;
		_own_address = 0x00;
		_scl_gpio_pin = GPIO_Pin_8;
		_sda_gpio_pin = GPIO_Pin_7;
		_gpio_chanel = GPIOB;
		_i2c_chanel = I2C1;
	}
	
	void initialize();
	void setClockSpeed(uint32_t clock_size)	{_clock_speed = clock_size;}
	void setMode(uint16_t which_mode) {_i2c_mode = which_mode;}
	void setDutyCycle(uint16_t which_duty_mode) {_duty_cycle = which_duty_mode ;}
	void setOwnAddress(uint16_t own_address) {_own_address = own_address;}
	
	int writeArrary(uint8_t w_addr, const uint8_t* p_data, uint16_t len);
	int writeByte(uint8_t w_addr, const uint8_t* p_data);
	int readByte(uint8_t r_addr, uint8_t* p_data, uint16_t len);
	int readArrary();
	int cleanReadBuf();
	
	void setFastMode();
	void setStandardMode();
	
private:
	void releaseBusByForce(void);
	uint32_t I2Cx_TIMEOUT_UserCallback(char value);
	uint32_t WAIT_FOR_FLAG(uint32_t flag, FlagStatus value, int timeout, int errorcode);

	I2C_InitTypeDef _i2c;
	I2C_TypeDef * _i2c_chanel;
	GPIO_InitTypeDef _gpio;
	GPIO_TypeDef * _gpio_chanel;

	uint16_t _scl_gpio_pin;
	uint16_t _sda_gpio_pin;
	uint32_t _clock_speed;
	uint16_t _i2c_mode;
	uint16_t _duty_cycle;
	uint16_t _own_address;
	
};

#endif
