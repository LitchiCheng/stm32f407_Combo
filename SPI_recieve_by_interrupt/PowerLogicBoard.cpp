#include "PowerLogicBoard.h"
#include "Console.h"
#include "Timer.h"
#include "MessageTask.h"
#include "CUsart.h"
#include "GlobalData.h"
#include "CommandDispatchTask.h"

#define POWER_MANAGER_SPI   SPI2
#define POWER_MANAGER_SPI_NSS_PORT          GPIOB
#define POWER_MANAGER_SPI_SCK_PORT          GPIOB
#define POWER_MANAGER_SPI_MISO_PORT         GPIOB
#define POWER_MANAGER_SPI_MOSI_PORT         GPIOB
#define POWER_MANAGER_SPI_NSS_PIN           GPIO_Pin_9
#define POWER_MANAGER_SPI_MISO_PIN			GPIO_Pin_10
#define POWER_MANAGER_SPI_SCK_PIN			GPIO_Pin_14
#define POWER_MANAGER_SPI_MOSI_PIN			GPIO_Pin_15
#define POWER_MANAGER_SPI_IRQ				SPI2_IRQn

namespace {	
	Timer CheckPowerManagertimer(50,50);
	const uint8_t RX_BUFFER_SIZE = 8;
	uint8_t SPI_SLAVE_Buffer_Rx[RX_BUFFER_SIZE] = {};
	uint16_t buffer_pointer = 0;
}

CPowerLogicBoard::CPowerLogicBoard()
{
	memset(SPI_SLAVE_Buffer_Rx,0,RX_BUFFER_SIZE);
	bRcvFinished = false;
}

int CPowerLogicBoard::doInit()
{
	initSPI();
	setOpen(true);
	return 0;
}

int CPowerLogicBoard::doRun()
{
	if (CheckPowerManagertimer.isAbsoluteTimeUp())	
	{
		if(GlobalData::OK)	Console::Instance()->printf("\r\n %d, %d, %d, %d, %d \r\n",SPI_SLAVE_Buffer_Rx[0] , SPI_SLAVE_Buffer_Rx[1], SPI_SLAVE_Buffer_Rx[2], SPI_SLAVE_Buffer_Rx[3], SPI_SLAVE_Buffer_Rx[7]);
	}
	return 0;
}

void CPowerLogicBoard::initSPI()
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	GPIO_InitStructure.GPIO_Pin    = POWER_MANAGER_SPI_NSS_PIN | POWER_MANAGER_SPI_MISO_PIN | POWER_MANAGER_SPI_SCK_PIN | POWER_MANAGER_SPI_MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_NOPULL;
	GPIO_Init(POWER_MANAGER_SPI_NSS_PORT, &GPIO_InitStructure); 
	/*!< PinAFConfig */
	//GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_SPI2);  
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_SPI2);  
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2); 
 
    /* SPI2 configuration */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;////
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//// 
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(POWER_MANAGER_SPI, &SPI_InitStructure);

    SPI_I2S_ITConfig(POWER_MANAGER_SPI, SPI_I2S_IT_RXNE, ENABLE);
	SPI_Cmd(POWER_MANAGER_SPI, ENABLE);			
	
	NVIC_InitTypeDef NVIC_InitStructure;	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = POWER_MANAGER_SPI_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

int queryLastPowerStateRecordHandler(uint8_t*, uint16_t)
{
	CmdSocket::Instance()->transParam(0) = CmdSocket::Instance()->lastCmdTyp();
	for(int i = 0; i < RX_BUFFER_SIZE; i++ )
	{
		CmdSocket::Instance()->transParam(i+1) = SPI_SLAVE_Buffer_Rx[i];
	}
	CmdSocket::Instance()->sendto(RX_BUFFER_SIZE * sizeof(uint32_t), CmdSocket::Instance()->cmderIP(), CmdSocket::Instance()->cmderPort());
	return 0;
}

#ifdef __cplusplus
extern "C" {
#endif 
void SPI2_IRQHandler(void)
{
	if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) == SET)
	{
		/* Receive Transaction data */
		if (0x01 == (uint8_t)SPI_I2S_ReceiveData(SPI2))
		{
			SPI_SLAVE_Buffer_Rx[0] = 0x01;
			buffer_pointer = 0;
		}
		SPI_SLAVE_Buffer_Rx[buffer_pointer] = SPI_I2S_ReceiveData(SPI2);
		buffer_pointer++;
		if(buffer_pointer >= RX_BUFFER_SIZE)
		{
			buffer_pointer = 0;
		}
	}
}

#ifdef __cplusplus
}
#endif 

