#include "I2CDriver.h"

static GPIO_InitTypeDef gpioinit;
static I2C_InitTypeDef hi2c1;

void I2CDriver::initialize()
{
	/* Enable the remapping of Pins 6/7 to 8/9 and the I2C clock before the initialize*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	gpioinit.GPIO_Mode = GPIO_Mode_OUT;
	gpioinit.GPIO_OType = GPIO_OType_PP;
	gpioinit.GPIO_PuPd = GPIO_PuPd_UP;
	gpioinit.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_7;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioinit);
	GPIO_SetBits(GPIOB, GPIO_Pin_7);
	GPIO_SetBits(GPIOB, GPIO_Pin_8);
	I2C_SoftwareResetCmd(I2C1, ENABLE);

	
	gpioinit.GPIO_Mode = GPIO_Mode_AF;
	gpioinit.GPIO_OType = GPIO_OType_OD;
	gpioinit.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_7;
	gpioinit.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpioinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_I2C1);
	GPIO_Init(GPIOB, &gpioinit);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
    hi2c1.I2C_ClockSpeed = 100000;
    hi2c1.I2C_Mode = I2C_Mode_I2C;
    hi2c1.I2C_DutyCycle = I2C_DutyCycle_16_9;
    hi2c1.I2C_OwnAddress1 = 0;
    hi2c1.I2C_Ack = I2C_Ack_Enable;
    hi2c1.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Cmd(I2C1, ENABLE);
	I2C_Init(I2C1, &hi2c1);
 
}