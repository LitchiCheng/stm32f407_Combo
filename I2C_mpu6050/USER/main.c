#include "I2CDriver.h"
#include "stm32f4xx.h"

#include "usart.h"
#include "delay.h"
#include "SEGGER_RTT.h"
#include <stdio.h>
#include "IMU.h"

#include "Timer.h"

/*******************************/
#include<math.h>

//---------------------------------------------------------------------------------------------------

 
float Yaw,Pitch,Roll;  //偏航角，俯仰角，翻滚


void LED_D2_D3(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIOInit;
	GPIOInit.GPIO_Mode = GPIO_Mode_OUT;
	GPIOInit.GPIO_OType = GPIO_OType_PP;
	GPIOInit.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIOInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOInit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIOInit);
	GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);
}

//void InitMPU6050(void);

//unsigned int GetData(unsigned char REG_Address);

//#define	SMPLRT_DIV		0x19	
//#define	CONFIG			0x1A	
//#define	GYRO_CONFIG		0x1B	
//#define	ACCEL_CONFIG	0x1C	
//#define	ACCEL_XOUT_H	0x3B
//#define	ACCEL_XOUT_L	0x3C
//#define	ACCEL_YOUT_H	0x3D
//#define	ACCEL_YOUT_L	0x3E
//#define	ACCEL_ZOUT_H	0x3F
//#define	ACCEL_ZOUT_L	0x40
//#define	TEMP_OUT_H		0x41
//#define	TEMP_OUT_L		0x42
//#define	GYRO_XOUT_H		0x43
//#define	GYRO_XOUT_L		0x44	
//#define	GYRO_YOUT_H		0x45
//#define	GYRO_YOUT_L		0x46
//#define	GYRO_ZOUT_H		0x47
//#define	GYRO_ZOUT_L		0x48
//#define	PWR_MGMT_1		0x6B	
//#define	WHO_AM_I		0x75	
//#define	SlaveAddress	0xD0

static Timer test1;
static Timer test2;

//MPU的send的地址是0xD0,不需要移位操作。出错的时候要等待I2C事件，不要重启总线，否则会一直卡在那。
int main(void)
{
	
	float ypr[3]; // yaw pitch roll
	uart_init(115200);
	delay_init(168);
	i2c::Instance()->initialize();
	//InitMPU6050();
	IMU_init();
	delay_ms(2000);
	LED_D2_D3();
	bool Turn = false;
	bool Turn1 = false;
	SEGGER_RTT_printf(0,"\r\n gyro start \r\n");
	MPU6050_InitGyro_Offset();
	while(1)
	{
		IMU_getYawPitchRoll(ypr);
//		printf2("\r\n---------aX--------%d \r\n",GetData(ACCEL_XOUT_H));
//		printf2("\r\n---------aY---------%d \r\n",GetData(ACCEL_YOUT_H));		
//		printf2("\r\n---------aZ----------%d \r\n",GetData(ACCEL_ZOUT_H));
//		
//		printf2("\r\n---------gX----------%f \r\n",GetData(GYRO_XOUT_H)/16.4); 
//		printf2("\r\n---------gY----------%f \r\n",GetData(GYRO_YOUT_H)/16.4); 
//		printf2("\r\n---------gZ----------%f \r\n",GetData(GYRO_ZOUT_H)/16.4);
		delay_ms(20);
//		printf2("\r\n now clock is %d \r\n",timer_lower::Instance()->getNowTime_ms());
		printf2("\r\n yaw is %0.3f, pitch is %0.3f, roll is %0.3f \r\n",ypr[0], ypr[1], ypr[2]);
		if(test1.isTimeUp_ms(1000))
		{
			Turn = Turn ? false:true;
			Turn ? GPIO_SetBits(GPIOA, GPIO_Pin_6) : GPIO_ResetBits(GPIOA, GPIO_Pin_6);
		}
		if(test2.isTimeUp_ms(1000))
		{
			Turn1 = Turn1 ? false:true;
			Turn1 ? GPIO_SetBits(GPIOA, GPIO_Pin_7) : GPIO_ResetBits(GPIOA, GPIO_Pin_7);
		}
	} 	
}




