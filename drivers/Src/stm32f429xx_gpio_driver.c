/*
 * stm32f429xx_gpio_driver.c
 *
 *  Created on: Nov 7, 2024
 *      Author: Shin
 */
#include "stm32f429xx_gpio_driver.h"


RCC_RegDef_t* pRCC = RCC;
void __LNH_GPIO_PeriClockControl(GPIO_RegDef_t*pGPIOx, uint8_t EN_DI)
{
	if (EN_DI == EN)
		{
			if (pGPIOx == GPIOA)
			{
				GPIOA_PCLK_EN();
			}else if (pGPIOx == GPIOB)
			{
				GPIOB_PCLK_EN();
			}else if (pGPIOx == GPIOC)
			{
				GPIOC_PCLK_EN();
			}else if (pGPIOx == GPIOD)
			{
				GPIOD_PCLK_EN();
			}else if (pGPIOx == GPIOE)
			{
				GPIOE_PCLK_EN();
			}else if (pGPIOx == GPIOF)
			{
				GPIOF_PCLK_EN();
			}else if (pGPIOx == GPIOG)
			{
				GPIOG_PCLK_EN();
			}else if (pGPIOx == GPIOH)
			{
				GPIOH_PCLK_EN();
			}else if (pGPIOx == GPIOI)
			{
				GPIOI_PCLK_EN();
			}else if (pGPIOx == GPIOJ)
			{
				GPIOJ_PCLK_EN();
			}else if (pGPIOx == GPIOK)
			{
				GPIOK_PCLK_EN();
			}

		}
		else
		{
			if (pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DI();
			}else if (pGPIOx == GPIOB)
			{
				GPIOB_PCLK_DI();
			}else if (pGPIOx == GPIOC)
			{
				GPIOC_PCLK_DI();
			}else if (pGPIOx == GPIOD)
			{
				GPIOD_PCLK_DI();
			}else if (pGPIOx == GPIOE)
			{
				GPIOE_PCLK_DI();
			}else if (pGPIOx == GPIOF)
			{
				GPIOF_PCLK_DI();
			}else if (pGPIOx == GPIOG)
			{
				GPIOG_PCLK_DI();
			}else if (pGPIOx == GPIOH)
			{
				GPIOH_PCLK_DI();
			}else if (pGPIOx == GPIOI)
			{
				GPIOI_PCLK_DI();
			}else if (pGPIOx == GPIOJ)
			{
				GPIOJ_PCLK_DI();
			}else if (pGPIOx == GPIOK)
			{
				GPIOK_PCLK_DI();
			}
		}

}

void __LNH_GPIO_Init(GPIO_Config_t* pGPIOConfig_t)
{
	/*uint32_t GPIO_PIN_L;
	uint32_t GPIO_PIN_H;

	if ( pGPIOConfig_t->pGPIOx == GPIOA )
	{
		if( pGPIOConfig_t->GPIO_PIN == GPIO_PIN_0 )
		{
		//Xác nhận 2 bit xác định cho PIN
		 GPIO_PIN_L = 2*(GPIO_PIN_0);
		 GPIO_PIN_H = 2*(GPIO_PIN_0)+1;
			//Cấu hình MODER
			if(pGPIOConfig_t->GPIO_MODE == GPIO_MODE_IN)
			{
					pGPIOConfig_t->pGPIOx->MODER &= ~(1<<(GPIO_PIN_L));
					pGPIOConfig_t->pGPIOx->MODER &= ~(1<<(GPIO_PIN_H));
			}else if(pGPIOConfig_t->GPIO_MODE == GPIO_MODE_OUT)
			{
					pGPIOConfig_t->pGPIOx->MODER |=  (1<<(GPIO_PIN_L));
					pGPIOConfig_t->pGPIOx->MODER &= ~(1<<(GPIO_PIN_H));
			}else if(pGPIOConfig_t->GPIO_MODE == GPIO_MODE_ALT)
			{
				pGPIOConfig_t->pGPIOx->MODER &= ~(1<<(GPIO_PIN_L));
				pGPIOConfig_t->pGPIOx->MODER |=  (1<<(GPIO_PIN_H));
			}else if(pGPIOConfig_t->GPIO_MODE == GPIO_MODE_ANL)
			{
				pGPIOConfig_t->pGPIOx->MODER &= ~(1<<(GPIO_PIN_L));
				pGPIOConfig_t->pGPIOx->MODER |= ~(1<<(GPIO_PIN_H));
			}

		}else if(pGPIOConfig_t->GPIO_PIN == GPIO_PIN_1)
		{
			//Xác nhận 2 bit xác định cho PIN
			 GPIO_PIN_L = 2*(GPIO_PIN_0);
			 GPIO_PIN_H = 2*(GPIO_PIN_0)+1;
				//Cấu hình MODER
				if(pGPIOConfig_t->GPIO_MODE == GPIO_MODE_IN)
				{
						pGPIOConfig_t->pGPIOx->MODER &= ~(1<<(GPIO_PIN_L));
						pGPIOConfig_t->pGPIOx->MODER &= ~(1<<(GPIO_PIN_H));
				}else if(pGPIOConfig_t->GPIO_MODE == GPIO_MODE_OUT)
				{
						pGPIOConfig_t->pGPIOx->MODER |=  (1<<(GPIO_PIN_L));
						pGPIOConfig_t->pGPIOx->MODER &= ~(1<<(GPIO_PIN_H));
				}else if(pGPIOConfig_t->GPIO_MODE == GPIO_MODE_ALT)
				{
					pGPIOConfig_t->pGPIOx->MODER &= ~(1<<(GPIO_PIN_L));
					pGPIOConfig_t->pGPIOx->MODER |=  (1<<(GPIO_PIN_H));
				}else if(pGPIOConfig_t->GPIO_MODE == GPIO_MODE_ANL)
				{
					pGPIOConfig_t->pGPIOx->MODER &= ~(1<<(GPIO_PIN_L));
					pGPIOConfig_t->pGPIOx->MODER |= ~(1<<(GPIO_PIN_H));
				}
		}else if(pGPIOConfig_t->GPIO_PIN == GPIO_PIN_2)
		{
			//Xác nhận 2 bit xác định cho PIN
			 GPIO_PIN_L = 2*(GPIO_PIN_0);
			 GPIO_PIN_H = 2*(GPIO_PIN_0)+1;
				//Cấu hình MODER
				if(pGPIOConfig_t->GPIO_MODE == GPIO_MODE_IN)
				{
						pGPIOConfig_t->pGPIOx->MODER &= ~(1<<(GPIO_PIN_L));
						pGPIOConfig_t->pGPIOx->MODER &= ~(1<<(GPIO_PIN_H));
				}else if(pGPIOConfig_t->GPIO_MODE == GPIO_MODE_OUT)
				{
						pGPIOConfig_t->pGPIOx->MODER |=  (1<<(GPIO_PIN_L));
						pGPIOConfig_t->pGPIOx->MODER &= ~(1<<(GPIO_PIN_H));
				}else if(pGPIOConfig_t->GPIO_MODE == GPIO_MODE_ALT)
				{
					pGPIOConfig_t->pGPIOx->MODER &= ~(1<<(GPIO_PIN_L));
					pGPIOConfig_t->pGPIOx->MODER |=  (1<<(GPIO_PIN_H));
				}else if(pGPIOConfig_t->GPIO_MODE == GPIO_MODE_ANL)
				{
					pGPIOConfig_t->pGPIOx->MODER &= ~(1<<(GPIO_PIN_L));
					pGPIOConfig_t->pGPIOx->MODER |= ~(1<<(GPIO_PIN_H));
				}
		}*/
	/*Vì PIN quyết định vị trí ghi thông qua 2 biến GPIO_PIN_L và GPIO_PIN_H, vì vậy
	 * thay vì ta chia từng trường hợp PIN, kiểm tra xem trường hợp nào trùng với PIN
	 * mà người dùng nhập, rồi cấu hình từng cái, thì bây giờ ta láy số PIN người dùng nhập
	 * đem đi dịch làm địa chỉ luôn.
	 */
		//Tối ưu hóa code lần 1 :

		//Xác nhận 2 bit xác định cho PIN
	/*	uint32_t GPIO_PIN_L = 2 * (pGPIOConfig_t->GPIO_PIN);
	    uint32_t GPIO_PIN_H = GPIO_PIN_L + 1;

	    // Kiểm tra cổng GPIO
	    if (pGPIOConfig_t->pGPIOx == GPIOA)
	    {
	        // Cấu hình MODER cho từng chế độ
	        switch (pGPIOConfig_t->GPIO_MODE)
	        {
	            case GPIO_MODE_IN:  // Input mode
	                pGPIOConfig_t->pGPIOx->MODER &= ~(1 << GPIO_PIN_L);  // Clear bit L
	                pGPIOConfig_t->pGPIOx->MODER &= ~(1 << GPIO_PIN_H);  // Clear bit H
	                break;

	            case GPIO_MODE_OUT:  // Output mode
	                pGPIOConfig_t->pGPIOx->MODER |=  (1 << GPIO_PIN_L);  // Set bit L
	                pGPIOConfig_t->pGPIOx->MODER &= ~(1 << GPIO_PIN_H);  // Clear bit H
	                break;

	            case GPIO_MODE_ALT:  // Alternate function mode
	                pGPIOConfig_t->pGPIOx->MODER &= ~(1 << GPIO_PIN_L);  // Clear bit L
	                pGPIOConfig_t->pGPIOx->MODER |=  (1 << GPIO_PIN_H);  // Set bit H
	                break;

	            case GPIO_MODE_ANL:  // Analog mode
	                pGPIOConfig_t->pGPIOx->MODER |=  (1 << GPIO_PIN_L);  // Set bit L
	                pGPIOConfig_t->pGPIOx->MODER |=  (1 << GPIO_PIN_H);  // Set bit H
	                break;

	            default:
	                // Xử lý trường hợp không hợp lệ (nếu cần)
	                break;
	        }
	    }else if(pGPIOConfig_t->pGPIOx == GPIOB)
	    {
	        // Cấu hình MODER cho từng chế độ
	        switch (pGPIOConfig_t->GPIO_MODE)
	        {
	            case GPIO_MODE_IN:  // Input mode
	                pGPIOConfig_t->pGPIOx->MODER &= ~(1 << GPIO_PIN_L);  // Clear bit L
	                pGPIOConfig_t->pGPIOx->MODER &= ~(1 << GPIO_PIN_H);  // Clear bit H
	                break;

	            case GPIO_MODE_OUT:  // Output mode
	                pGPIOConfig_t->pGPIOx->MODER |=  (1 << GPIO_PIN_L);  // Set bit L
	                pGPIOConfig_t->pGPIOx->MODER &= ~(1 << GPIO_PIN_H);  // Clear bit H
	                break;

	            case GPIO_MODE_ALT:  // Alternate function mode
	                pGPIOConfig_t->pGPIOx->MODER &= ~(1 << GPIO_PIN_L);  // Clear bit L
	                pGPIOConfig_t->pGPIOx->MODER |=  (1 << GPIO_PIN_H);  // Set bit H
	                break;

	            case GPIO_MODE_ANL:  // Analog mode
	                pGPIOConfig_t->pGPIOx->MODER |=  (1 << GPIO_PIN_L);  // Set bit L
	                pGPIOConfig_t->pGPIOx->MODER |=  (1 << GPIO_PIN_H);  // Set bit H
	                break;

	            default:
	                // Xử lý trường hợp không hợp lệ (nếu cần)
	                break;
	        }
	    }*/
	//Tối ưu hóa code lần 2:
	/*Vì dù cho có cổng nào thì quá trình thực hiện vẫn như nhau, chỉ khác nhau ở thanh ghi
	 * trỏ đến, tuy nhiên trong lúc chúng ta trỏ đến các bit của thanh ghi, ta vô tình
	 * xác định luôn cổng cho nó vì cú pháp
	 * pGPIOConfig_t->pGPIOx->MODER, đã xác định pGPIOx do người dùng truyền vào.
	 */
	// Xác định 2 bit xác định cho chân GPIO
		uint32_t GPIO_PIN_LH1 = (pGPIOConfig_t->GPIO_PIN);
	    uint32_t GPIO_PIN_L2 = 2 * (pGPIOConfig_t->GPIO_PIN);
	    uint32_t GPIO_PIN_H2 = GPIO_PIN_L2 + 1;
	    uint32_t GPIO_PIN_LMA3 = 4 * ((pGPIOConfig_t->GPIO_PIN) % 8);
		uint32_t GPIO_PIN_LMI3 = GPIO_PIN_LMA3 + 1;
		uint32_t GPIO_PIN_HMI3 = GPIO_PIN_LMI3 + 1;
		uint32_t GPIO_PIN_HMA3 = GPIO_PIN_HMI3 + 1;
	    // Cấu hình MODER cho từng chế độ
	    switch (pGPIOConfig_t->GPIO_MODE)
	    {
	        case GPIO_MODE_IN:  // Input mode
	            pGPIOConfig_t->pGPIOx->MODER &= ~(1 << GPIO_PIN_L2);  // Clear bit L
	            pGPIOConfig_t->pGPIOx->MODER &= ~(1 << GPIO_PIN_H2);  // Clear bit H
	            break;

	        case GPIO_MODE_OUT:  // Output mode
	            pGPIOConfig_t->pGPIOx->MODER |=  (1 << GPIO_PIN_L2);  // Set bit L
	            pGPIOConfig_t->pGPIOx->MODER &= ~(1 << GPIO_PIN_H2);  // Clear bit H
	            break;

	        case GPIO_MODE_ALT:  // Alternate function mode
	            pGPIOConfig_t->pGPIOx->MODER &= ~(1 << GPIO_PIN_L2);  // Clear bit L
	            pGPIOConfig_t->pGPIOx->MODER |=  (1 << GPIO_PIN_H2);  // Set bit H
	            break;

	        case GPIO_MODE_ANL:  // Analog mode
	            pGPIOConfig_t->pGPIOx->MODER |=  (1 << GPIO_PIN_L2);  // Set bit L
	            pGPIOConfig_t->pGPIOx->MODER |=  (1 << GPIO_PIN_H2);  // Set bit H
	            break;

	        default:
	            // Xử lý trường hợp không hợp lệ (nếu cần)
	            break;
	    }
	    //Port được xác định qua ->pGPIOx, PIN được xác định qua GPIO_PIN_L và H


	    //Cấu hình SPEED cho từng chế độ
	    switch (pGPIOConfig_t->GPIO_SPEED)
		{
			case GPIO_SPEED_LS:  // Input mode
				pGPIOConfig_t->pGPIOx->OSPEEDR &= ~(1 << GPIO_PIN_L2);  // Clear bit L
				pGPIOConfig_t->pGPIOx->OSPEEDR &= ~(1 << GPIO_PIN_H2);  // Clear bit H
				break;

			case GPIO_SPEED_MS:  // Output mode
				pGPIOConfig_t->pGPIOx->OSPEEDR |=  (1 << GPIO_PIN_L2);  // Set bit L
				pGPIOConfig_t->pGPIOx->OSPEEDR &= ~(1 << GPIO_PIN_H2);  // Clear bit H
				break;

			case GPIO_SPEED_HS:  // Alternate function mode
				pGPIOConfig_t->pGPIOx->OSPEEDR &= ~(1 << GPIO_PIN_L2);  // Clear bit L
				pGPIOConfig_t->pGPIOx->OSPEEDR |=  (1 << GPIO_PIN_H2);  // Set bit H
				break;

			case GPIO_SPEED_VHS:  // Analog mode
				pGPIOConfig_t->pGPIOx->OSPEEDR |=  (1 << GPIO_PIN_L2);  // Set bit L
				pGPIOConfig_t->pGPIOx->OSPEEDR |=  (1 << GPIO_PIN_H2);  // Set bit H
				break;

			default:
				// Xử lý trường hợp không hợp lệ (nếu cần)
				break;
		}

	    //Cấu hình OTYPE cho từng chế độ
	    switch (pGPIOConfig_t->GPIO_OTYPE)
	    {
	    	case GPIO_OTYPE_PP:	//Output push pull mode
	    		pGPIOConfig_t->pGPIOx->OTYPER &= ~(1 << GPIO_PIN_LH1);  // Clear bit LH1
	    		break;

	    	case GPIO_OTYPE_OD: 	//Output open drain mode
	    		pGPIOConfig_t->pGPIOx->OTYPER |=  (1 << GPIO_PIN_LH1);  // Set bit LH1
	    		break;

	    	default:
	    		// Xử lí trường hợp không hợp lệ (nếu cần)
	    		break;
	    }
	    //Cấu hình PUPD cho từng chế độ
	    switch (pGPIOConfig_t->GPIO_PUPD)
	    {
	    	case GPIO_PUPD_NP: //No pull up and no pull down
	    		pGPIOConfig_t->pGPIOx->PUPDR &= ~(1 << GPIO_PIN_L2);  // Clear bit L
	    		pGPIOConfig_t->pGPIOx->PUPDR &= ~(1 << GPIO_PIN_H2);  // Clear bit H
	    		break;

	    	case GPIO_PUPD_PU: //Pull up
	    		pGPIOConfig_t->pGPIOx->PUPDR |=  (1 << GPIO_PIN_L2);  // Set bit L
	    		pGPIOConfig_t->pGPIOx->PUPDR |= ~(1 << GPIO_PIN_H2);  // Clear bit H
	    		break;

	    	case GPIO_PUPD_PD:  // Alternate function mode
	    		pGPIOConfig_t->pGPIOx->PUPDR &= ~(1 << GPIO_PIN_L2);  // Clear bit L
	    		pGPIOConfig_t->pGPIOx->PUPDR |=  (1 << GPIO_PIN_H2);  // Set bit H
	    		break;

	    	case GPIO_PUPD_RV:  // Analog mode
	    		pGPIOConfig_t->pGPIOx->PUPDR |=  (1 << GPIO_PIN_L2);  // Set bit L
	    		pGPIOConfig_t->pGPIOx->PUPDR |=  (1 << GPIO_PIN_H2);  // Set bit H
	    		break;

	    	default:
	    		// Xử lí trường hợp không hợp lệ (nếu cần)
	    		break;

	    }
	    // Cấu hình AF cho từng chế độ
	    if(pGPIOConfig_t->GPIO_PIN <=7)
	    {
		    switch (pGPIOConfig_t->GPIO_AF)
		    {
		    	case GPIO_AF_0:
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_1:
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_2:
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_3:
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_4:
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_5:
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_6:
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_7:
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_8:
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_9:
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_10:
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_11:
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_12:
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_13:
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_14:
		    		pGPIOConfig_t->pGPIOx->AFRL &=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_15:
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRL |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	default:
		    		// Xử lí trường hợp không hợp lệ (nếu cần)
		    		break;

		    }
	    }
	    else
	    {
		    switch (pGPIOConfig_t->GPIO_AF)
		    {
		    	case GPIO_AF_0:
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_1:
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_2:
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_3:
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_4:
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_5:
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_6:
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_7:
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_8:
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_9:
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_10:
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_11:
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_12:
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_13:
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH &= ~(1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_14:
		    		pGPIOConfig_t->pGPIOx->AFRH &=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	case GPIO_AF_15:
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMA3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_LMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_HMI3);
		    		pGPIOConfig_t->pGPIOx->AFRH |=  (1 << GPIO_PIN_HMA3);
		    		break;

		    	default:
		    		// Xử lí trường hợp không hợp lệ (nếu cần)
		    		break;

		    }
	    }
	    if(pGPIOConfig_t->GPIO_MODE <=3)
	    {
	    	uint8_t temp;
	    	temp = (pGPIOConfig_t->GPIO_MODE) << (2*pGPIOConfig_t->GPIO_PIN);
	    	pGPIOConfig_t->pGPIOx->MODER &= ~(0x3 << (pGPIOConfig_t->GPIO_PIN));
	    	pGPIOConfig_t->pGPIOx->MODER |= temp;
	    }
	    else
	    {
			//Cấu hình ngắt cho từng chế độ
			switch(pGPIOConfig_t->EXTI_TRIGGER)
			{

					case EXTI_IT_FT: //Configure FTSR

						pGPIOConfig_t->pEXTI->FTSR |=	(1 << GPIO_PIN_LH1);
						pGPIOConfig_t->pEXTI->RTSR &=  ~(1 << GPIO_PIN_LH1);
						break;

					case EXTI_IT_RT: //Configure RTSR

						pGPIOConfig_t->pEXTI->RTSR |=	(1 << GPIO_PIN_LH1);
						pGPIOConfig_t->pEXTI->FTSR &=  ~(1 << GPIO_PIN_LH1);
						break;

					case EXTI_IT_RFT: //Configure RTSR

						pGPIOConfig_t->pEXTI->RTSR |=	(1 << GPIO_PIN_LH1);
						pGPIOConfig_t->pEXTI->FTSR |=   (1 << GPIO_PIN_LH1);
						break;
					default:
						// Xử lí trường hợp không hợp lệ (nếu cần)
						break;
			}
			//Cấu hình ngắt chọn chân
			uint8_t temp1 = (pGPIOConfig_t->GPIO_PIN)/4;
			uint8_t temp2 = (pGPIOConfig_t->GPIO_PIN)%4;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOConfig_t->pGPIOx);
			SYSCFG_PCLK_EN();
			pGPIOConfig_t->pSYSCFG->EXTICR[temp1] = portcode << (temp2 *4);

			//Cấu hình cho phép ngắt
			if (pGPIOConfig_t->EXTI_ENDI == EXTI_EN)
			{
				pGPIOConfig_t->pEXTI->IMR  |=  (1 << GPIO_PIN_LH1);
			}
			else
			{
				pGPIOConfig_t->pEXTI->IMR  &=  ~(1 << GPIO_PIN_LH1);
			}
	    }
}


void __LNH_GPIO_DeInit(GPIO_RegDef_t*pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}else if (pGPIOx == GPIOJ)
	{
		GPIOJ_REG_RESET();
	}else if (pGPIOx == GPIOK)
	{
		GPIOK_REG_RESET();
	}
}

uint8_t __LNH_GPIO_ReadPin(GPIO_RegDef_t*pGPIOx,uint8_t GPIO_PIN_NUMBER)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> GPIO_PIN_NUMBER) &0x00000001);
	//Bitwise để đọc 1 bit
	return value;
}
uint16_t __LNH_GPIO_ReadPort(GPIO_RegDef_t*pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

void __LNH_GPIO_WritePin(GPIO_RegDef_t*pGPIOx,uint8_t GPIO_PIN_NUMBER,uint8_t value)
{
	if (value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << GPIO_PIN_NUMBER);
		//Bitwise để ghi bit 1 vào 1 vị trí
	}
	else
	{
		pGPIOx->ODR &= ~(1 << GPIO_PIN_NUMBER);
		//Bitwise để ghi bit 0 vào 1 vị trí
	}
}

void __LNH_GPIO_WritePort(GPIO_RegDef_t*pGPIOx,uint16_t value)
{
	pGPIOx->ODR = value;
}

void __LNH_GPIO_TogglePin(GPIO_RegDef_t*pGPIOx,uint8_t GPIO_PIN_NUMBER)
{
	pGPIOx->ODR ^= (1 << GPIO_PIN_NUMBER );
}

void __LNH_GPIO_IRQ_ITConfig(uint8_t IRQNumber ,uint8_t EN_DI)
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
void __LNH_GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8*iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENT);
	*(NVIC_PR_BASE_ADDR+(iprx*4)) |= (IRQPriority << shift_amount );
}
void __LNH_GPIO_IRQHanding(uint8_t GPIO_PIN_NUMBER)
{
	EXTI_RegDef_t* pEXTI = EXTI;
	//Clear the EXTI bit PR to pending to waiting for I
	if (pEXTI->PR & (1 << GPIO_PIN_NUMBER))
	{
		pEXTI->PR |= (1 << GPIO_PIN_NUMBER);
	}
}

