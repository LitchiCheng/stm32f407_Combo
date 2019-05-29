#ifndef _POWER_LOGIC_BOARD_H_
#define _POWER_LOGIC_BOARD_H_

#include "Singleton.h"
#include "Device.h"
#include "Reporter.h"

class CPowerLogicBoard: public CDevice, public CReporter_base
{
	public:
		CPowerLogicBoard();
		virtual int doInit();
		virtual int doRun();
		void initSPI();
		void receiveData(uint8_t temp);
		int queryLastPowerStateRecordHandler(uint8_t*, uint16_t);
	private:
		bool bRcvFinished;
		int report();
};
typedef NormalSingleton<CPowerLogicBoard> PowerLogicBoard;
#endif

