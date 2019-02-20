#ifndef	__I2CDriver__H 
#define __I2CDriver__H

#include "stm32f4xx.h"
class I2CDriver
{
	public:
		I2CDriver(){}
		void initialize();
		void setClock();
		void setSpeed();
		int writeArrary();
		int writeByte();
		int readByte();
		int readArrary();
		int cleanReadBuf();
		~I2CDriver(){}
	private:
		void releaseBusByForce();
		
};

#endif