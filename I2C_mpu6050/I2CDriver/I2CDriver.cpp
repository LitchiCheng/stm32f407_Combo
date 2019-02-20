#include "I2CDriver.h"

static GPIO_InitTypeDef gpioinit;
static I2C_InitTypeDef hi2c1;

#define I2Cx_FLAG_TIMEOUT			((uint32_t) 900)
#define I2Cx_LONG_TIMEOUT 			((uint32_t)(300 * I2Cx_FLAG_TIMEOUT))
#define WAIT_FOR_FLAG(flag, value, timeout, errorcode)	I2CTimeout = timeout;\
														while(I2C_GetFlagStatus(I2C1, flag) != value){\
														if((I2CTimeout--) == 0) return I2Cx_TIMEOUT_UserCallback(errorcode);\
														}\

#define CLEAR_ADDR_BIT			I2C_ReadRegister(I2C1, I2C_Register_SR1);\
								I2C_ReadRegister(I2C1, I2C_Register_SR2);\

static uint32_t I2Cx_TIMEOUT_UserCallback(char value)
{
	I2C_InitTypeDef hi2c1;
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_SoftwareResetCmd(I2C1, ENABLE);
	I2C_SoftwareResetCmd(I2C1, DISABLE);
	I2C_DeInit(I2C1);
	hi2c1.I2C_ClockSpeed = 100000;
    hi2c1.I2C_Mode = I2C_Mode_I2C;
    hi2c1.I2C_DutyCycle = I2C_DutyCycle_2;
    hi2c1.I2C_OwnAddress1 = 0;
    hi2c1.I2C_Ack = I2C_Ack_Enable;
    hi2c1.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Cmd(I2C1, ENABLE);
	I2C_Init(I2C1, &hi2c1);
	//Console::Instance()->printf("I2C1 Restarted. \n");
	return 1;
}

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

int I2CDriver::writeArrary(uint8_t w_addr, const uint8_t* p_data, uint16_t len)
{	
	uint16_t i = 0;
	__IO uint32_t I2CTimeout = I2Cx_LONG_TIMEOUT;
	WAIT_FOR_FLAG(I2C_FLAG_BUSY,RESET,I2Cx_LONG_TIMEOUT,1);	
	I2C_GenerateSTART(I2C1, ENABLE);
	WAIT_FOR_FLAG(I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 2);
	I2C_Send7bitAddress(I2C1,(w_addr << 1), I2C_Direction_Transmitter);
	WAIT_FOR_FLAG(I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 3);
	CLEAR_ADDR_BIT;
	while (i < len) 
	{
		I2C_SendData(I2C1, *p_data);
		WAIT_FOR_FLAG(I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 4);
		i++; p_data++;
	}
	WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 5);
	I2C_GenerateSTOP(I2C1, ENABLE);
	return 0;
}

int I2CDriver::writeByte(uint8_t w_addr, const uint8_t* p_data)
{
	return writeArrary(w_addr, p_data, 1);
}

int I2CDriver::readByte(uint8_t r_addr, uint8_t* p_data, uint16_t len)
{
	__IO uint32_t I2CTimeout = I2Cx_LONG_TIMEOUT;
	WAIT_FOR_FLAG(I2C_FLAG_BUSY,RESET,I2Cx_LONG_TIMEOUT,6);
	I2C_GenerateSTART(I2C1, ENABLE);
	WAIT_FOR_FLAG(I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 7);
	I2C_Send7bitAddress(I2C1, (r_addr << 1), I2C_Direction_Transmitter);
	WAIT_FOR_FLAG(I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 8);
	CLEAR_ADDR_BIT;
	WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 9);
	I2C_SendData(I2C1,r_addr);
	WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 10); 
	I2C_GenerateSTART(I2C1, ENABLE);
	WAIT_FOR_FLAG (I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 11);
	I2C_Send7bitAddress(I2C1,(r_addr << 1), I2C_Direction_Receiver);
	WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 12);
	/*规定死就2个长度*/
	while(len) 
	{
		if (len == 1)
		{
			I2C_AcknowledgeConfig(I2C1, DISABLE);
			I2C_GenerateSTOP(I2C1, ENABLE);
		}
		if (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			*p_data = I2C_ReceiveData(I2C1);
			p_data++;
			len--;
		}
	}
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	return 0;
} 
