#include "I2CDriver.h"
#include "stm32f4xx.h"

#include "usart.h"
#include "delay.h"
//#include <stdint.h>
//#include "I2CDriver.h"
#include "SEGGER_RTT.h"
void LED_D2_D3(uint32_t);

int main(void)
{
	
	u32 t=0;
	uart_init(115200);
	delay_init(84);
	//LED_D2_D3();
	LED_D2_D3(GPIO_Pin_7);
	GPIO_SetBits(GPIOA, GPIO_Pin_7);
	uint8_t sendDATA = 0x55;
	I2CDriver i2c;
	i2c.initialize();
	LED_D2_D3(GPIO_Pin_6);
	while (1) 
	{
		//SEGGER_RTT_printf(0,"xxxxxx\r\n");
		i2c.writeByte(0x02, &sendDATA);
		GPIO_SetBits(GPIOA, GPIO_Pin_6);
		delay_ms(2000);
		GPIO_ResetBits(GPIOA, GPIO_Pin_6);
		delay_ms(2000);
      /* printf("SHT sensor probing failed\n"); */
    }
    delay_ms(2000);
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



