/*
 * stm32f429xx_spi_driver.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Shin
 */

#include "stm32f429xx_spi_driver.h"

static void SPI_TXE_IT_HANDLE(SPI_Config_t* pSPIConfig);
static void SPI_RXE_IT_HANDLE(SPI_Config_t* pSPIConfig);
static void SPI_OVR_ERR_IT_HANDLE(SPI_Config_t* pSPIConfig);
void __LNH_SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t EN_DI)
{
	if (EN_DI == EN)
			{
				if (pSPIx == SPI1)
				{
					SPI1_PCLK_EN();
				}else if (pSPIx == SPI2)
				{
					SPI2_PCLK_EN();
				}else if (pSPIx == SPI3)
				{
					SPI3_PCLK_EN();
				}else if (pSPIx == SPI4)
				{
					SPI4_PCLK_EN();
				}else if (pSPIx == SPI5)
				{
					SPI5_PCLK_EN();
				}else if (pSPIx == SPI6)
				{
					SPI6_PCLK_EN();
				}

			}
			else
			{
				if (pSPIx == SPI1)
				{
					SPI1_PCLK_DI();
				}else if (pSPIx == SPI2)
				{
					SPI2_PCLK_DI();
				}else if (pSPIx == SPI3)
				{
					SPI3_PCLK_DI();
				}else if (pSPIx == SPI4)
				{
					SPI4_PCLK_DI();
				}else if (pSPIx == SPI5)
				{
					SPI5_PCLK_DI();
				}else if (pSPIx == SPI6)
				{
					SPI6_PCLK_DI();
				}
			}
}

void __LNH_SPI_Init(SPI_Config_t* pSPIConfig)
{
	//	I. Configure the device mode
	if (pSPIConfig->SPI_DeviceMode == SPI_DEVICE_MODE_MASTER)
	{
		pSPIConfig->pSPIx->CR1 |=  (1 << 2);
	}
	else
	{
		pSPIConfig->pSPIx->CR1 &= ~(1 << 2);
	}
	//	II. Configure the bus configure
	switch (pSPIConfig->SPI_BusConfig)
	{
		case SPI_BUS_CONFIG_FD: //Full Duplex Mode
			pSPIConfig->pSPIx->CR1 &= ~(1<<15);
			break;
		case SPI_BUS_CONFIG_HD: //Half Duplex Mode
			pSPIConfig->pSPIx->CR1 |=  (1<<15);
			break;

		case SPI_BUS_CONFIG_SIMPLEX_TXONLY: //Only TX
			pSPIConfig->pSPIx->CR1 &= ~(1<<15);
			pSPIConfig->pSPIx->CR1 &= ~(1<<10);
			break;
		case SPI_BUS_CONFIG_SIMPLEX_RXONLY: //Only RX
			pSPIConfig->pSPIx->CR1 &= ~(1<<15);
			pSPIConfig->pSPIx->CR1 |=  (1<<10);
			break;

	}
	/* Lấy giá trị người gửi gửi vào truyền trực tiếp vào luôn, khỏi cần ghi từng bit.
	 * Điều này làm được khi ta define đúng với giá trị cần đưa vào thanh ghi.
	 * Nhưng viết 1 nhóm bit vào mà ta chỉ sử dụng 1 phép OR (kĩ thuật truyền bit 1)
	 * Thì bit 0 sẽ không đúng, do vậy ta cần clear bit hết thanh ghi trước khi truyền
	 * vào.
	 */
	//	III. Configure the SPI Speed
	pSPIConfig->pSPIx->CR1 &= ~(7<<3);
	pSPIConfig->pSPIx->CR1 |= ((pSPIConfig->SPI_SCLKSpeed)<<3);

	//	IV. Configure the DFF
	pSPIConfig->pSPIx->CR1 &= ~(1<<11);
	pSPIConfig->pSPIx->CR1 |= ((pSPIConfig->SPI_SCLKSpeed)<<11);

	//	V. Configure the CPOL
	pSPIConfig->pSPIx->CR1 &= ~(1<<1);
	pSPIConfig->pSPIx->CR1 |= ((pSPIConfig->SPI_CPOL)<<1);

	// 	VI. Configure the CPHA

	pSPIConfig->pSPIx->CR1 &= ~(1<<0);
	pSPIConfig->pSPIx->CR1 |= ((pSPIConfig->SPI_CPHA)<<0);

	//	VII. Configure the SSM
	if (pSPIConfig->SPI_SSM == SPI_SSM_SW)
	{
		pSPIConfig->pSPIx->CR1 |= (1<<9);
	}
	else
	{
		pSPIConfig->pSPIx->CR1 &= ~(1<<9);
	}
	//	VII. Configure the NSS

	if (pSPIConfig->SPI_SSM == SPI_SSM_SW)
	{
		if (pSPIConfig->SPI_FUNC == SPI_FUNC_MT)
		{
			pSPIConfig->pSPIx->CR1 |= (1<<8);
		}
		else
		{
			pSPIConfig->pSPIx->CR1 &= ~(1<<8);
		}
	}
	else
	{
		if (pSPIConfig->SPI_FUNC == SPI_FUNC_MT)
		{
			pSPIConfig->pSPIx->CR2 |=  (1<<2);
		}
		else
		{
			pSPIConfig->pSPIx->CR2 &= ~(1<<2);
		}
		/* Cả 2 cách đều có cấu hình thanh ghi như nhau.
		 * Với cách dùng NSS, thì nối chân NSS vào CS của slave, SPI tự đưa xuống thấp
		 * Với cách dùng GPIO, thì nối chân NSS lên nguồn rồi dùng GPIO điều khiển.
		 */
	}
}


void __LNH_SPI_DeInit(SPI_RegDef_t*pSPIx)
{
	if (pSPIx == SPI1)
		{
			SPI1_REG_RESET();
		}else if (pSPIx == SPI2)
		{
			SPI2_REG_RESET();
		}else if (pSPIx == SPI3)
		{
			SPI3_REG_RESET();
		}else if (pSPIx == SPI4)
		{
			SPI4_REG_RESET();
		}else if (pSPIx == SPI5)
		{
			SPI5_REG_RESET();
		}else if (pSPIx == SPI6)
		{
			SPI6_REG_RESET();
		}
}
uint8_t __LNH_SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
uint8_t __LNH_SPI_BSY(SPI_RegDef_t* pSPIx)
{
	uint8_t BSY =  (((pSPIx->SR)>>7)&1);
	return BSY;
}

void __LNH_SPI_SendData(SPI_RegDef_t* pSPIx,uint8_t* pTxBuffer, uint32_t Length)
{
	while (Length >0)
	{
		//1. Wait until TXE is set
		while(!(pSPIx->SR &(1<<1)));
		//2. Check the DFF bit in CR1
		if (pSPIx->CR1 &(1<<11))
		{
			// 16 bit
			pSPIx->DR =  *((uint16_t*)pTxBuffer);
			Length--;
			Length--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8 bit
			pSPIx->DR = *pTxBuffer;
			Length--;
			pTxBuffer++;
		}
	}
}

void __LNH_SPI_ReceiveData(SPI_RegDef_t* pSPIx,uint8_t* pRxBuffer, uint32_t Length)
{
	while (Length >0)
	{
		//1. Wait until RXNE is set
		while(!(pSPIx->SR &(1<<0)));
		//2. Check the DFF bit in CR1
		if (pSPIx->CR1 &(1<<11))
		{
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Length--;
			Length--;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			*pRxBuffer = pSPIx->DR;
			Length--;
			pRxBuffer++;
		}
	}
}
void __LNH_SPI_PeripheralENDI(SPI_RegDef_t* pSPIx, uint8_t EN_DI)
{
	if (EN_DI == EN)
	{
		pSPIx->CR1 |=  (1<<6);
	}
	else
	{
		pSPIx->CR1 &= ~(1<<6);
	}
}
void __LNH_SPI_IRQ_ITConfig(uint8_t IRQNumber ,uint8_t EN_DI)
{
	if(EN_DI == EN)
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= ( 1 << (IRQNumber%32) );
		}else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER3 |= ( 1 << (IRQNumber%64) );
		}
	}else
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= ( 1 << (IRQNumber%32) );
		}else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER3 |= ( 1 << (IRQNumber%64) );
		}
	}
}
void __LNH_SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8*iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENT);
	*(NVIC_PR_BASE_ADDR+(iprx*4)) |= (IRQPriority << shift_amount );
}
uint8_t __LNH_SPI_SendDataIT(SPI_Config_t* pSPIConfig,uint8_t* pTxBuffer, uint32_t Length)
{

	uint8_t state = pSPIConfig->TxState;
	if (state != SPI_BUSY_IN_TX)
	{
	//1. Save the TxBuffer address and Len information in some global var
		pSPIConfig->pTxBuffer = pTxBuffer;
		pSPIConfig->TxLen = Length;
	/*2. Mark the SPI state as busy in transmission so that no other code can take over
	 * same SPI peripherals until transmission is over */
		pSPIConfig->TxState = SPI_BUSY_IN_TX;

	/*3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR*/
		pSPIConfig->pSPIx->CR2 |= (1 << 7);
	}
	return state;

}
uint8_t __LNH_SPI_ReceiveDataIT(SPI_Config_t* pSPIConfig,uint8_t* pRxBuffer, uint32_t Length)
{
	uint8_t state = pSPIConfig->RxState;
	if (state != SPI_BUSY_IN_RX)
	{
	//1. Save the TxBuffer address and Len information in some global var
		pSPIConfig->pRxBuffer = pRxBuffer;
		pSPIConfig->RxLen = Length;
	/*2. Mark the SPI state as busy in transmission so that no other code can take over
	 * same SPI peripherals until transmission is over */
		pSPIConfig->RxState = SPI_BUSY_IN_RX;

	/*3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR*/
	pSPIConfig->pSPIx->CR2 |= (1 << 6);
	}
	return state;
}

void __LNH_SPI_IRQHanding(SPI_Config_t* pSPIConfig)
{
	uint8_t temp1, temp2 ;
	// First lets check for TXE
	temp1 = pSPIConfig->pSPIx->SR & (1 << 1);
	temp2 = pSPIConfig->pSPIx->CR2 & (1 << 7);
	if (temp1 && temp2)
	{
		//Handle TXE
		SPI_TXE_IT_HANDLE(pSPIConfig);

	}
	// Second lets check for RXNE
	temp1 = pSPIConfig->pSPIx->SR & (1 << 0);
	temp2 = pSPIConfig->pSPIx->CR2 & (1 << 6);
	if (temp1 && temp2)
	{
		//Handle RXNE
		SPI_RXE_IT_HANDLE(pSPIConfig);

	}
	// Third lets check for OVR Flag
	temp1 = pSPIConfig->pSPIx->SR & (1 << 6);
	temp2 = pSPIConfig->pSPIx->CR2 & (1 << 5);
	if (temp1 && temp2)
	{
		//Handle OVR Error
		SPI_OVR_ERR_IT_HANDLE(pSPIConfig);

	}

}



// Some helper function implementations
static void SPI_TXE_IT_HANDLE(SPI_Config_t* pSPIConfig)
{
	// Check the DFF bit in CR1
			if (pSPIConfig->pSPIx->CR1 &(1<<11))
			{
				// 16 bit
				pSPIConfig ->pSPIx->DR = *((uint16_t*)pSPIConfig->pTxBuffer);
				pSPIConfig->TxLen--;
				pSPIConfig->TxLen--;
				(uint16_t*)pSPIConfig->pTxBuffer ++;

			}
			else
			{
				// 8 bit
				pSPIConfig->pSPIx->DR = *pSPIConfig-> pTxBuffer;
				pSPIConfig->TxLen--;
				pSPIConfig->pTxBuffer++;
			}
			if(! pSPIConfig->TxLen)
			{
				/*TxLen is zero, so close the SPI transmission and inform the application
				 * that Tx is over*/
				/*This prevent interrupt more than 2 interrupt*/
				SPI_CloseTransmisson(pSPIConfig);
				__LNH_ApplicationEventCallback(pSPIConfig,SPI_EVENT_TX_CMPLT);
			}
}


static void SPI_RXE_IT_HANDLE(SPI_Config_t* pSPIConfig)
{
	// Check the DFF bit in CR1
			if (pSPIConfig->pSPIx->CR1 &(1<<11))
			{
				// 16 bit
				*((uint16_t*)pSPIConfig->pRxBuffer) = (uint16_t)pSPIConfig->pSPIx->DR;
				pSPIConfig->RxLen-=2;
				pSPIConfig->pRxBuffer--;
				pSPIConfig->pRxBuffer--;

			}
			else
			{
				// 8 bit
				*(pSPIConfig->pRxBuffer) = (uint8_t)pSPIConfig->pSPIx->DR;
				pSPIConfig->RxLen--;
				pSPIConfig->pRxBuffer--;
			}
			if(!pSPIConfig->RxLen)
			{
				/*TxLen is zero, so close the SPI transmission and inform the application
				 * that Tx is over*/
				/*This prevent interrupt more than 2 interrupt*/
				SPI_CloseReception(pSPIConfig);
				__LNH_ApplicationEventCallback(pSPIConfig,SPI_EVENT_RX_CMPLT);
			}
}

static void SPI_OVR_ERR_IT_HANDLE(SPI_Config_t* pSPIConfig)
{

	uint8_t temp;
	//1. Clear the OVR Flag
	if (pSPIConfig->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIConfig->pSPIx->DR;
		temp = pSPIConfig->pSPIx->SR;
	}
	(void)temp;
	//2. Inform the application
	__LNH_ApplicationEventCallback(pSPIConfig,SPI_EVENT_OVR_ERR);



}

void SPI_CloseTransmisson(SPI_Config_t* pSPIConfig)
{
	pSPIConfig->pSPIx->CR2 &= ~(1<<7);
	pSPIConfig->pTxBuffer = NULL;
	pSPIConfig->TxLen = 0;
	pSPIConfig->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Config_t* pSPIConfig)
{
	pSPIConfig->pSPIx->CR2 &= ~(1<<6);
	pSPIConfig->pRxBuffer = NULL;
	pSPIConfig->RxLen = 0;
	pSPIConfig->RxState = SPI_READY;
}
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}
__weak void __LNH_ApplicationEventCallback(SPI_Config_t* pSPIConfig, uint8_t AppEV)
{
	/* This is a weak implementation. The application may override this function */
}










