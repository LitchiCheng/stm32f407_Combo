#include "Timer.h"
#include "SEGGER_RTT.h"

#define BASIC_TIM TIM6
#define BASIC_TIM_CLK RCC_APB1Periph_TIM6

#define BASIC_TIM_IRQn TIM6_DAC_IRQn
#define BASIC_TIM_IRQHandler TIM6_DAC_IRQHandler

/****************************************************************************************************************/
Timer::Timer():_previous_time_ms(0), _previous_time_us(0)
{

}

bool Timer::isTimeUp_us(uint32_t how_long_us)
{
    if(_first_come_for_us)
    {
        _first_come_for_us = false;
        _previous_time_us = timer_lower::Instance()->getNowTime_us();
    }
    if((timer_lower::Instance()->getNowTime_us() - _previous_time_us) >= how_long_us)
	{
		_first_come_for_us = true;
        return true;
    }
	else
    {
        return false;
    }  
}

bool Timer::isTimeUp_ms(uint32_t how_long_ms)
{
    if(_first_come_for_ms)
    {
        _first_come_for_ms = false;
        _previous_time_ms = timer_lower::Instance()->getNowTime_ms();
    }
    if((timer_lower::Instance()->getNowTime_ms() - _previous_time_ms) >= how_long_ms)
	{
		_first_come_for_ms = true;
        return true;
    }
	else
    {
        return false;
    } 
}

void Timer::delay_ms(uint32_t how_long_ms)
{
	uint32_t start_time = timer_lower::Instance()->getNowTime_ms();
	while(!((timer_lower::Instance()->getNowTime_ms()-start_time) >= how_long_ms)){}
}

void Timer::delay_us(uint32_t how_long_us)
{
	uint32_t start_time = timer_lower::Instance()->getNowTime_us();
	while(!((timer_lower::Instance()->getNowTime_us()-start_time) >= how_long_us)){}
}

/****************************************************************************************************************/
CTimer_Lower::CTimer_Lower()
{
	//lower part need to implement
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(BASIC_TIM_CLK, ENABLE);

	//1 ms/cycle
	TIM_TimeBaseStructure.TIM_Period = 10-1;

	//1 us/count
	TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;
	
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(BASIC_TIM, &TIM_TimeBaseStructure);

	TIM_ClearFlag(BASIC_TIM, TIM_FLAG_Update);

	TIM_ITConfig(BASIC_TIM,TIM_IT_Update,ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;

	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(BASIC_TIM, ENABLE);
}

uint32_t CTimer_Lower::getNowTime_us()
{
    //lower part need to implement
	return TIM_GetCounter(BASIC_TIM);
}

#ifdef __cplusplus
 extern "C" {
#endif
void TIM6_DAC_IRQHandler(void)
{
	if ( TIM_GetITStatus( BASIC_TIM, TIM_IT_Update) == SET ) 
	{
		TIM_ClearITPendingBit(BASIC_TIM , TIM_IT_Update);
		timer_lower::Instance()->nowTimeUpdate_ms();
		TIM_ClearITPendingBit(BASIC_TIM , TIM_IT_Update);				
	}
	
}
#ifdef __cplusplus
}
#endif

/****************************************************************************************************************/