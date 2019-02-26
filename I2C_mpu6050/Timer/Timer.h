#ifndef __Timer_H
#define __Timer_H
#include "stm32f4xx.h"
#include "Singleton.h"

class Timer
{
public:
	Timer();
    ~Timer(){}
	bool isTimeUp_us(uint32_t how_long_us);
    bool isTimeUp_ms(uint32_t how_long_ms);
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

    uint32_t _previous_time_us;
    uint32_t _previous_time_ms;
    uint32_t _later_time_us;
    uint32_t _later_time_ms;
    uint32_t _last_for_how_long_us;
    uint32_t _now_time_ms;
    bool _first_come_for_ms;
    bool _first_come_for_us;
};

typedef NormalSingleton<Timer> timer;

#endif