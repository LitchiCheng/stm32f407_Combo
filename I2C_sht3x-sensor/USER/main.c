#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include <stdint.h>
void LED_D2_D3(uint32_t);
#include "SEGGER_RTT.h"
#include "sht.h"

//PB7 - SDA
//PB8 - SCL
//GND - VSS
//3.3V - VCC

int main(void)
{
	
	u32 t=0;
	uart_init(115200);
	delay_init(84);
	//LED_D2_D3();
	//LED_D2_D3(GPIO_Pin_7);
  while (sht_probe() != STATUS_OK) {
	  LED_D2_D3(GPIO_Pin_6);
        SEGGER_RTT_printf(0,"SHT sensor probing failed\n");
    }
	GPIO_SetBits(GPIOA, GPIO_Pin_6);
	LED_D2_D3(GPIO_Pin_7);
    SEGGER_RTT_printf(0,"SHT sensor probing successful\n");

    while (1) {
        s32 temperature, humidity;
        /* Measure temperature and relative humidity and store into variables
         * temperature, humidity (each output multiplied by 1000).
         */
        s8 ret = sht_measure_blocking_read(&temperature, &humidity);
        if (ret == STATUS_OK) {
            SEGGER_RTT_printf(0,"measured temperature: %d degreeCelsius, "
                      "measured humidity: %d percentRH\n",
                      temperature / 1000,
                      humidity / 1000); 
			LED_D2_D3(GPIO_Pin_6);
        } else {
            //printf("error reading measurement\n");
			LED_D2_D3(GPIO_Pin_7);
        }

        delay_ms(1000);
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



