#ifndef __Timer_H
#define __Timer_H
#include "stm32f4xx.h"
#include "Singleton.h"

class CTimer_Lower
{
public:
	CTimer_Lower();
	~CTimer_Lower();
	uint32_t getNowTime_us();
    uint32_t nowTimeUpdate_ms()
    {
        _now_time_ms++;
    }
    uint32_t getNowTime_ms()
    {
        return _now_time_ms;
    }
private:
	uint32_t _now_time_ms;
};

typedef NormalSingleton<CTimer_Lower> timer_lower;

class Timer
{
public:
	Timer();
    ~Timer();
	bool isTimeUp_us(uint32_t how_long_us);
    bool isTimeUp_ms(uint32_t how_long_ms);
	void delay_ms(uint32_t how_long_ms);
	void delay_us(uint32_t how_long_us);
private:

    uint32_t _previous_time_us;
    uint32_t _previous_time_ms;
    uint32_t _later_time_us;
    uint32_t _later_time_ms;

    bool _first_come_for_ms;
    bool _first_come_for_us;
};




#endif