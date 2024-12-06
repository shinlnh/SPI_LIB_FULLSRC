/*
 * stm32f429xx_gpio_driver.h
 *
 *  Created on: Nov 7, 2024
 *      Author: Shin
 */

#ifndef INC_STM32F429XX_GPIO_DRIVER_H_
#define INC_STM32F429XX_GPIO_DRIVER_H_

#include "stm32f429xx.h"


/************************STRUCT FOR CONFIG Handle GPIO***********************************/

//Define GPIO_PIN_NUMBER

#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

//define GPIO_PIN_MODE

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALT		2
#define GPIO_MODE_ANL		3
#define GPIO_MODE_IT		4




//define GPIO_PIN_SPEED

#define GPIO_SPEED_LS		0
#define GPIO_SPEED_MS		1
#define GPIO_SPEED_HS		2
#define GPIO_SPEED_VHS		3

//define GPIO_PIN_OTyper

#define GPIO_OTYPE_PP		0
#define GPIO_OTYPE_OD		1


//define GPIO_PIN_PUPDR

#define GPIO_PUPD_NP		0
#define GPIO_PUPD_PU		1
#define GPIO_PUPD_PD		2
#define GPIO_PUPD_RV		3

//define GPIO_PIN_AF

#define GPIO_AF_0			0
#define GPIO_AF_1			1
#define GPIO_AF_2			2
#define GPIO_AF_3			3
#define GPIO_AF_4			4
#define GPIO_AF_5			5
#define GPIO_AF_6			6
#define GPIO_AF_7			7
#define GPIO_AF_8			8
#define GPIO_AF_9			9
#define GPIO_AF_10			10
#define GPIO_AF_11			11
#define GPIO_AF_12			12
#define GPIO_AF_13			13
#define GPIO_AF_14			14
#define GPIO_AF_15			15


//EXTI EXTI_ENDI
#define EXTI_DI				0
#define EXTI_EN				1

//define EXTI_IT_TRIGGER

#define EXTI_IT_FT			1
#define EXTI_IT_RT			2
#define EXTI_IT_RFT			3


typedef struct
{
	GPIO_RegDef_t* pGPIOx;
	/*Khi ta viết như thế này, GPIO_RegDef_t ở trên vẫn không có địa chỉ, nhưng pGPIOx lại có
	 * địa chỉ, việc khai báo này sẽ sao chép các kiểu dữ liệu (KDL không phải dữ liệu nên không
	 * tốn bộ nhớ) vào pGPIOx, và do pGPIOx là 1 biến nên các KDL đó có dữ liệu và địa chỉ được
	 * tính vào địa chỉ base của pGPIOx.
	 * Tóm lại, việc chọn con trỏ struct chứ không phải struct thường là để định địa chỉ ban đầu
	 * cho việc sao chép struct cho trùng với các địa chỉ thanh ghi.
	 * Thay vì tạo ra tất cả các biến pGPIOA,B,C,... để trỏ đến từng ADDR base thanh ghi
	 * GPIO thì ta dùng pGPIOx này bỏ vào if else để người dùng cần GPIO nào thì tạo ra
	 * biến pGPIO đó, việc này tiết kiệm bộ nhớ và gọn hơn nhiều.
	 */
	uint8_t GPIO_PIN;
	uint8_t GPIO_MODE;
	uint8_t GPIO_SPEED;
	uint8_t GPIO_PUPD;
	uint8_t GPIO_OTYPE;
	uint8_t GPIO_AF;
	/*Các biến uint8_t này không dùng để viết vào thanh ghi như pGPIOx, mà nó chỉ dùng để
	 * lưu giá trị tạm thời người dùng cấu hình để truyền vào các thanh ghi đã được thiết
	 * lập địa chỉ trong GPIO_RegDef_t
	 */
	EXTI_RegDef_t*  pEXTI;
	SYSCFG_RegDef_t* pSYSCFG;
	uint8_t EXTI_ENDI;
	uint8_t EXTI_TRIGGER;
}GPIO_Config_t;



/************************API FOR CONFIG HANDLE GPIO**************************************/
//Peripheral Clock Setup
void __LNH_GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EN_DI);


//Init And DeInit Peripheral
void __LNH_GPIO_Init(GPIO_Config_t* pGPIOConfig);
void __LNH_GPIO_DeInit(GPIO_RegDef_t*pGPIOx);

/*Data Read And Write*/
//Data Read :
	//Đọc 1 bit của 1 chân
uint8_t __LNH_GPIO_ReadPin(GPIO_RegDef_t*pGPIOx,uint8_t GPIO_PIN_NUMBER);



	//Đọc tất cả 16 bit của 1 port
uint16_t __LNH_GPIO_ReadPort(GPIO_RegDef_t*pGPIOx);


//Data Write :
	//Viết 1 bit vào 1 chân
void __LNH_GPIO_WritePin(GPIO_RegDef_t*pGPIOx,uint8_t GPIO_PIN_NUMBER,uint8_t value);


	//Viết 16 bit vào 1 port
void __LNH_GPIO_WritePort(GPIO_RegDef_t*pGPIOx,uint16_t value);



void __LNH_GPIO_TogglePin(GPIO_RegDef_t*pGPIOx,uint8_t GPIO_PIN_NUMBER);


//IRQ Configuration and ISR handing
void __LNH_GPIO_IRQ_ITConfig(uint8_t IRQNumber,uint8_t EN_DI);
void __LNH_GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void __LNH_GPIO_IRQHanding(uint8_t GPIO_PIN_NUMBER);




#endif /* INC_STM32F429XX_GPIO_DRIVER_H_ */
