#include "I2CDriver.h"
#include "stm32f4xx.h"

#include "usart.h"
#include "delay.h"
#include "SEGGER_RTT.h"

void LED_D2_D3(uint32_t);

void InitMPU6050(void);

unsigned int GetData(unsigned char REG_Address);

#define	SMPLRT_DIV		0x19	
#define	CONFIG			0x1A	
#define	GYRO_CONFIG		0x1B	
#define	ACCEL_CONFIG	0x1C	
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	
#define	WHO_AM_I		0x75	
#define	SlaveAddress	0xD0

I2CDriver i2c;

//MPU的send的地址是0xD0,不需要移位操作。出错的时候要等待I2C事件，不要重启总线，否则会一直卡在那。
int main(void)
{
	uart_init(115200);
	delay_init(84);
	i2c.initialize();
	InitMPU6050();
	delay_ms(2000);
	SEGGER_RTT_printf(0,"\r\n gyro start \r\n");
	while(1)
	{
		SEGGER_RTT_printf(0,"\r\n---------aX--------%d \r\n",GetData(ACCEL_XOUT_H));
		SEGGER_RTT_printf(0,"\r\n---------aY---------%d \r\n",GetData(ACCEL_YOUT_H)); 
		SEGGER_RTT_printf(0,"\r\n---------aZ----------%d \r\n",GetData(ACCEL_ZOUT_H)); 
		SEGGER_RTT_printf(0,"\r\n---------gX----------%d \r\n",GetData(GYRO_XOUT_H)); 
		SEGGER_RTT_printf(0,"\r\n---------gY----------%d \r\n",GetData(GYRO_YOUT_H)); 
		SEGGER_RTT_printf(0,"\r\n---------gZ----------%d \r\n",GetData(GYRO_ZOUT_H));
		delay_ms(2000);
		SEGGER_RTT_printf(0,"=====================================================================");
	} 
	
}

void LED_D2_D3(uint32_t GPIO_Pin_x)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIOInit;
	GPIOInit.GPIO_Mode = GPIO_Mode_OUT;
	GPIOInit.GPIO_OType = GPIO_OType_PP;
	GPIOInit.GPIO_Pin = GPIO_Pin_x;
	GPIOInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOInit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIOInit);
}

void InitMPU6050(void)
{
	i2c.I2C_ByteWrite(PWR_MGMT_1,0x80);			//初始化之前一定要复位以下，再唤醒，延时时间还要够长，否则没用。
	
	delay_ms(2000);
	
	i2c.I2C_ByteWrite(PWR_MGMT_1,0x00);

	i2c.I2C_ByteWrite(SMPLRT_DIV,0x07);

	i2c.I2C_ByteWrite(CONFIG,0x06);

	i2c.I2C_ByteWrite(GYRO_CONFIG,0x18);

	i2c.I2C_ByteWrite(ACCEL_CONFIG,0x01);
}

unsigned int GetData(unsigned char REG_Address)
{
	char H,L;
	H=i2c.I2C_ByteRead(REG_Address);
	L=i2c.I2C_ByteRead(REG_Address+1);
	return (H<<8)+L;   
}


