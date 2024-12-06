/*
 * stm32f429xx_spi_driver.h
 *
 *  Created on: Nov 15, 2024
 *      Author: Shin
 */

#ifndef INC_STM32F429XX_SPI_DRIVER_H_
#define INC_STM32F429XX_SPI_DRIVER_H_

#include "stm32f429xx.h"
typedef struct
{
	SPI_RegDef_t* pSPIx;
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SCLKSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t	SPI_SSM;
	uint8_t SPI_FUNC;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t  TxState;
	uint8_t  RxState;
}SPI_Config_t;
//DEFINE FOR ALL SELECT OF SPI

//1. SPI DEVICE MODE

#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

//2. SPI BUS MODE
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY		3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		4


//3. SPI CLKSPEED


#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7


//4. SPI DFF

#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1



//5. SPI CLOCK

//a.  CPOL

#define SPI_CPOL_HIGH				1
#define SPI_CPOL_LOW				0

//b. CPHA

#define SPI_CPHA_HIGH				1
#define SPI_CPHA_LOW				0


//6. SPI SSM



#define SPI_SSM_HW					0
#define SPI_SSM_SW					1


//7. SPI Function

#define SPI_FUNC_MT					0
#define SPI_FUNC_SL					1

//8. SPI BUSY

#define SPI_READY 					0
#define SPI_BUSY_IN_RX				1
#define SPI_BUSY_IN_TX				2

//9. Possible SPI Application events
#define SPI_EVENT_TX_CMPLT			1
#define SPI_EVENT_RX_CMPLT			2
#define SPI_EVENT_OVR_ERR			3
#define SPI_EVENT_CRC_ERR			4
/******************************API FOR CONFIG HANDLE SPI*********************************/
//Peripheral Clock Setup
void __LNH_SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t EN_DI);


//Init And DeInit Peripheral
void __LNH_SPI_Init(SPI_Config_t* pSPIConfig);
void __LNH_SPI_DeInit(SPI_RegDef_t*pSPIx);


//Send And Receive

void __LNH_SPI_SendData(SPI_RegDef_t* pSPIx,uint8_t* pTxBuffer, uint32_t Length);
void __LNH_SPI_ReceiveData(SPI_RegDef_t* pSPIx,uint8_t* pRxBuffer, uint32_t Length);

//Send And Receive IT
uint8_t __LNH_SPI_SendDataIT(SPI_Config_t* pSPIx,uint8_t* pTxBuffer, uint32_t Length);
uint8_t __LNH_SPI_ReceiveDataIT(SPI_Config_t* pSPIx,uint8_t* pRxBuffer, uint32_t Length);

//IRQ Configuration and ISR handing
void __LNH_SPI_IRQ_ITConfig(uint8_t IRQNumber,uint8_t EN_DI);
void __LNH_SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void __LNH_SPI_IRQHanding(SPI_Config_t* pSPIConfig);


//Enable SPI

void __LNH_SPI_PeripheralENDI(SPI_RegDef_t* pSPIx, uint8_t EN_DI);


//Confirm SPI not busy

uint8_t __LNH_SPI_BSY(SPI_RegDef_t* pSPIx);

//Other Function In SPI
uint8_t __LNH_SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t* pSPIx);
void SPI_CloseTransmisson(SPI_Config_t* pSPIConfig);
void SPI_CloseReception(SPI_Config_t* pSPIConfig);

/* Application callback */

void __LNH_ApplicationEventCallback(SPI_Config_t* pSPIConfig, uint8_t AppEV);

#endif /* INC_STM32F429XX_SPI_DRIVER_H_ */
