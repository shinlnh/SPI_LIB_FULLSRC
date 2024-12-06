/*
 * stm32f429xx.h
 *
 *  Created on: Nov 4, 2024
 *      Author: Shin
 */

#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_


#include "stdint.h"
#include "stddef.h"

#define __vo volatile
#define __weak	__attribute__((weak))
/*************************ADDRESS BASE FOR ALL IN STM32F429XX********************/

//Base Address for ARM CORTEX MX Processor

#define NVIC_ISER0				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1				((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2				((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3				((__vo uint32_t*)0xE000E10C)

#define NVIC_ICER0				((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1				((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2				((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3				((__vo uint32_t*)0xE000E18C)


#define NVIC_PR_BASE_ADDR 		((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENT	4

//Base address of Flash and RAM memories

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR 			0x20000000U
#define SRAM2_BASEADDR			0x20001C00U
#define ROM_BASEADDR			0x1FFF0000U
#define SRAM 					SRAM1_BASEADDR

//Base address of AHB and APB Bus

#define PHERIPH_BASEADDR 			0x40000000U
#define APB1PERIPH_BASEADDR 		PHERIPH_BASEADDR
#define APB2PERIPH_BASEADDR 		0x40010000U
#define AHB1PERIPH_BASEADDR 		0x40020000U
#define AHB2PERIPH_BASEADDR 		0x50000000U

//Base address of peripheral which are hanging on AHB1 bus

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR+0x00000000U)
#define GPIOB_BASEADDR 				(AHB1PERIPH_BASEADDR+0x00000400U)
#define GPIOC_BASEADDR 				(AHB1PERIPH_BASEADDR+0x00000800U)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR+0x00000C00U)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR+0x00001000U)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR+0x00001400U)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR+0x00001800U)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR+0x00001C00U)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR+0x00002000U)
#define GPIOJ_BASEADDR				(AHB1PERIPH_BASEADDR+0x00002400U)
#define GPIOK_BASEADDR				(AHB1PERIPH_BASEADDR+0x00002800U)
#define CRC_BASEADDR				(AHB1PERIPH_BASEADDR+0x00003000U)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR+0x00003800U)
#define FLASH_IF_REG_BASEADDR		(AHB1PERIPH_BASEADDR+0x00003C00U)
#define BKPSRAM						(AHB1PERIPH_BASEADDR+0x00004000U)
#define DMA1_BASEADDR				(AHB1PERIPH_BASEADDR+0x00006000U)
#define DMA2_BASEADDR				(AHB1PERIPH_BASEADDR+0x00006400U)
#define ETHERNET_MAC_BASEADDR		(AHB1PERIPH_BASEADDR+0x00008000U)
#define DMA2D_BASEADDR				(AHB1PERIPH_BASEADDR+0x0000B000U)
#define USB_OTG_HS_BASEADDR			(AHB1PERIPH_BASEADDR+0x00020000U)

//Base address of peripheral which are hanging on AHB2 bus


#define USB_OTG_FS_BASEADDR         (AHB2PERIPH_BASEADDR + 0x00000000U)
#define DCMI_BASEADDR               (AHB2PERIPH_BASEADDR + 0x00050000U)
#define CRYP_BASEADDR               (AHB2PERIPH_BASEADDR + 0x00060000U)
#define HASH_BASEADDR               (AHB2PERIPH_BASEADDR + 0x00060400U)
#define RNG_BASEADDR                (AHB2PERIPH_BASEADDR + 0x00060800U)

// Base address of peripherals which are hanging on APB2 bus

#define TIM1_BASEADDR               (APB2PERIPH_BASEADDR + 0x00000000U)
#define TIM8_BASEADDR               (APB2PERIPH_BASEADDR + 0x00000400U)
#define USART1_BASEADDR             (APB2PERIPH_BASEADDR + 0x00001000U)
#define USART6_BASEADDR             (APB2PERIPH_BASEADDR + 0x00001400U)
#define ADC_BASEADDR                (APB2PERIPH_BASEADDR + 0x00002000U)
#define SDIO_BASEADDR               (APB2PERIPH_BASEADDR + 0x00002C00U)
#define SPI1_BASEADDR               (APB2PERIPH_BASEADDR + 0x00003000U)
#define SPI4_BASEADDR               (APB2PERIPH_BASEADDR + 0x00003400U)
#define SYSCFG_BASEADDR             (APB2PERIPH_BASEADDR + 0x00003800U)
#define EXTI_BASEADDR 				(APB2PERIPH_BASEADDR + 0x00003C00U)
#define TIM9_BASEADDR               (APB2PERIPH_BASEADDR + 0x00004000U)
#define TIM10_BASEADDR              (APB2PERIPH_BASEADDR + 0x00004400U)
#define TIM11_BASEADDR              (APB2PERIPH_BASEADDR + 0x00004800U)
#define SPI5_BASEADDR               (APB2PERIPH_BASEADDR + 0x00005000U)
#define SPI6_BASEADDR               (APB2PERIPH_BASEADDR + 0x00005400U)
#define SAI1_BASEADDR               (APB2PERIPH_BASEADDR + 0x00005800U)
#define LCD_TFT_BASEADDR            (APB2PERIPH_BASEADDR + 0x00006800U)


// Base address of peripherals which are hanging on APB1 bus

#define TIM2_BASEADDR               (APB1PERIPH_BASEADDR + 0x00000000U)
#define TIM3_BASEADDR               (APB1PERIPH_BASEADDR + 0x00000400U)
#define TIM4_BASEADDR               (APB1PERIPH_BASEADDR + 0x00000800U)
#define TIM5_BASEADDR               (APB1PERIPH_BASEADDR + 0x00000C00U)
#define TIM6_BASEADDR               (APB1PERIPH_BASEADDR + 0x00001000U)
#define TIM7_BASEADDR               (APB1PERIPH_BASEADDR + 0x00001400U)
#define TIM12_BASEADDR              (APB1PERIPH_BASEADDR + 0x00001800U)
#define TIM13_BASEADDR              (APB1PERIPH_BASEADDR + 0x00001C00U)
#define TIM14_BASEADDR              (APB1PERIPH_BASEADDR + 0x00002000U)
#define WWDG_BASEADDR               (APB1PERIPH_BASEADDR + 0x00002C00U)
#define IWDG_BASEADDR               (APB1PERIPH_BASEADDR + 0x00003000U)
#define SPI2_I2S2_BASEADDR          (APB1PERIPH_BASEADDR + 0x00003800U)
#define SPI3_I2S3_BASEADDR          (APB1PERIPH_BASEADDR + 0x00003C00U)
#define SPDIFRX_BASEADDR            (APB1PERIPH_BASEADDR + 0x00004000U)
#define USART2_BASEADDR             (APB1PERIPH_BASEADDR + 0x00004400U)
#define USART3_BASEADDR             (APB1PERIPH_BASEADDR + 0x00004800U)
#define UART4_BASEADDR              (APB1PERIPH_BASEADDR + 0x00004C00U)
#define UART5_BASEADDR              (APB1PERIPH_BASEADDR + 0x00005000U)
#define I2C1_BASEADDR               (APB1PERIPH_BASEADDR + 0x00005400U)
#define I2C2_BASEADDR               (APB1PERIPH_BASEADDR + 0x00005800U)
#define I2C3_BASEADDR               (APB1PERIPH_BASEADDR + 0x00005C00U)
#define CAN1_BASEADDR               (APB1PERIPH_BASEADDR + 0x00006400U)
#define CAN2_BASEADDR               (APB1PERIPH_BASEADDR + 0x00006800U)
#define PWR_BASEADDR                (APB1PERIPH_BASEADDR + 0x00007000U)
#define DAC_BASEADDR                (APB1PERIPH_BASEADDR + 0x00007400U)
#define UART7_BASEADDR              (APB1PERIPH_BASEADDR + 0x00007800U)
#define UART8_BASEADDR              (APB1PERIPH_BASEADDR + 0x00007C00U)

/********************************STRUCT FOR RCC***********************************/

/*Thay vì ta tiếp tục define từng địa chỉ thanh ghi thành phần của RCC dựa trên Base
 * address của RCC, thì ta sử dụng cách tăng địa chỉ của struct sẽ gọn hơn
 */

typedef struct
{

	__vo uint32_t RCC_CR;		//0x3800
	__vo uint32_t RCC_PLLCFGR;	//0x3804
	__vo uint32_t RCC_CFGR;		//0x3808
	__vo uint32_t RCC_CIR;		//0x380C
	__vo uint32_t RCC_AHB1RSTR;	//0x3810
	__vo uint32_t RCC_AHB2RSTR; //0x3814
	__vo uint32_t RCC_AHB3RSTR;	//0x3818
	__vo uint32_t RESERVED0;	//0x381C
	__vo uint32_t RCC_APB1RSTR;	//0x3820
	__vo uint32_t RCC_APB2RSTR; //0x3824
	__vo uint32_t RESERVED1;	//0x3828
	__vo uint32_t RESERVED2;	//0x382C
	__vo uint32_t RCC_AHB1ENR;	//0x3830
	__vo uint32_t RCC_AHB2ENR;	//...
	__vo uint32_t RCC_AHB3ENR;	//...
	__vo uint32_t RESERVED3;
	__vo uint32_t RCC_APB1ENR;
	__vo uint32_t RCC_APB2ENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t RESERVED5;
	__vo uint32_t RCC_AHB1LPENR;
	__vo uint32_t RCC_AHB2LPENR;
	__vo uint32_t RCC_AHB3LPENR;
	__vo uint32_t RESERVED6;
	__vo uint32_t RCC_APB1LPENR;
	__vo uint32_t RCC_APB2LPENR;
	__vo uint32_t RESERVED7;
	__vo uint32_t RESERVED8;
	__vo uint32_t RCC_BDCR;
	__vo uint32_t RCC_CSR;
	__vo uint32_t RESERVED9;
	__vo uint32_t RESERVED10;
	__vo uint32_t RCC_SSCGR;
	__vo uint32_t RCC_PLLI2SCFGR;
	__vo uint32_t RCC_PLLSAICFGR;
	__vo uint32_t RCC_DCKCFGR;

}RCC_RegDef_t;
/*Lúc này struct này không có địa chỉ, và cứ mỗi thành phần trong struct này
 * sẽ chiếm 4 byte bộ nhớ do kiểu dữ liệu uint32_t nếu struct đươc sao chép vào 1 biến.
 * Vì vậy, địa chỉ thành phần đứng dưới
 * sẽ bằng thành phần ở trên cộng với 4, trùng với khoảng cách offset của mỗi thanh ghi.
 * Để tất cả thanh ghi trùng với địa chỉ của nó, thì địa chỉ struct cũng là địa chỉ ban đầu
 * của thanh ghi đầu tiên phải trùng. Do đó ta phải gán địa chỉ struct thành địa chỉ base
 * của RCC.
 */
/*Sao chép 1 struct không địa chỉ ở trên (vì không có biến nên không tiêu hao bộ nhớ) vào
 * 1 biến để nó có địa chỉ ban đầu, và biến này phải có địa chỉ chính xác với địa chỉ base
 * RCC. Vì vậy ta sẽ sử dụng biến con trỏ struct.
 */
#define RCC					(RCC_RegDef_t*)RCC_BASEADDR

//RCC_RegDef_t*pRCC = RCC;

/********************************STRUCT FOR EXTI**********************************/

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

#define EXTI               (EXTI_RegDef_t*)EXTI_BASEADDR

/********************************STRUCT FOR SYSCFG**********************************/
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	__vo uint32_t RESERVED2[2];
	__vo uint32_t CFGR;

}SYSCFG_RegDef_t;

#define SYSCFG					(SYSCFG_RegDef_t*)SYSCFG_BASEADDR
/********************************STRUCT FOR GPIO**********************************/


/*Thay vì ta tiếp tục define từng địa chỉ thanh ghi thành phần của GPIO dựa trên Base
 * address của từng cổng GPIO, thì ta sử dụng cách tăng địa chỉ của struct sẽ gọn hơn
 */
typedef struct
{
	__vo uint32_t MODER;		//0x00 refer to ADDR Base pGPIOA
	__vo uint32_t OTYPER;		//0x04 refer to ADDR Base pGPIOA
	__vo uint32_t OSPEEDR;		//0x08 refer to ADDR Base pGPIOA
	__vo uint32_t PUPDR;		//...refer to ADDR Base pGPIOA
	__vo uint32_t IDR;			//...
	__vo uint32_t ODR;			//...
	__vo uint32_t BSRR;			//..
	__vo uint32_t LCKR;			//..
	__vo uint32_t AFRL;		    //..
	__vo uint32_t AFRH;		    //..
}GPIO_RegDef_t;
/*Lúc này struct này không có địa chỉ, và cứ mỗi thành phần trong struct này
 * sẽ chiếm 4 byte bộ nhớ do kiểu dữ liệu uint32_t nếu struct đươc sao chép vào 1 biến.
 * Vì vậy, địa chỉ thành phần đứng dưới
 * sẽ bằng thành phần ở trên cộng với 4, trùng với khoảng cách offset của mỗi thanh ghi.
 * Để tất cả thanh ghi trùng với địa chỉ của nó, thì địa chỉ struct cũng là địa chỉ ban đầu
 * của thanh ghi đầu tiên phải trùng. Do đó ta phải gán địa chỉ struct thành địa chỉ base
 * của GPIO.
 */
/*Sao chép 1 struct không địa chỉ ở trên (vì không có biến nên không tiêu hao bộ nhớ) vào
 * 1 biến để nó có địa chỉ ban đầu, và biến này phải có địa chỉ chính xác với địa chỉ base
 * GPIO. Vì vậy ta sẽ sử dụng biến con trỏ struct.
 */
#define GPIOA			 	(GPIO_RegDef_t*)GPIOA_BASEADDR
#define GPIOB			 	(GPIO_RegDef_t*)GPIOB_BASEADDR
#define GPIOC			 	(GPIO_RegDef_t*)GPIOC_BASEADDR
#define GPIOD			 	(GPIO_RegDef_t*)GPIOD_BASEADDR
#define GPIOE			 	(GPIO_RegDef_t*)GPIOE_BASEADDR
#define GPIOF			 	(GPIO_RegDef_t*)GPIOF_BASEADDR
#define GPIOG			 	(GPIO_RegDef_t*)GPIOG_BASEADDR
#define GPIOH			 	(GPIO_RegDef_t*)GPIOH_BASEADDR
#define GPIOI			 	(GPIO_RegDef_t*)GPIOI_BASEADDR
#define GPIOJ			 	(GPIO_RegDef_t*)GPIOJ_BASEADDR
#define GPIOK			 	(GPIO_RegDef_t*)GPIOK_BASEADDR

/*GPIO_RegDef_t* pGPIOA = GPIOA;
GPIO_RegDef_t* pGPIOB = GPIOB;
GPIO_RegDef_t* pGPIOC = GPIOC;
GPIO_RegDef_t* pGPIOD = GPIOD;
GPIO_RegDef_t* pGPIOE = GPIOE;
GPIO_RegDef_t* pGPIOF = GPIOF;
GPIO_RegDef_t* pGPIOG = GPIOG;
GPIO_RegDef_t* pGPIOH = GPIOH;
GPIO_RegDef_t* pGPIOI = GPIOI;
GPIO_RegDef_t* pGPIOJ = GPIOJ;
GPIO_RegDef_t* pGPIOK = GPIOK;*/
/*Khi ta viết như thế này, GPIO_RegDef_t ở trên vẫn không có địa chỉ, nhưng pGPIOx lại có
 * địa chỉ, việc khai báo này sẽ sao chép các kiểu dữ liệu (KDL không phải dữ liệu nên không
 * tốn bộ nhớ) vào pGPIOx, và do pGPIOx là 1 biến nên các KDL đó có dữ liệu và địa chỉ được
 * tính vào địa chỉ base của pGPIOx.
 * Tóm lại, việc chọn con trỏ struct chứ không phải struct thường là để định địa chỉ ban đầu
 * cho việc sao chép struct cho trùng với các địa chỉ thanh ghi.
 * Bước khai báo từng pointer để giữ vị trí này được chuyển sang bên gpio_driver.h, mục
 * đích để nhận giá trị người dùng cần tạo GPIO nào thì chỉ tạo con trỏ cho GPIO đó thôi
 * Tạo như thế này sẽ tốn bộ nhớ và dài dòng.
 */

/**************************STRUCT FOR SPI************************************************/

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;

#define SPI1 					(SPI_RegDef_t*)SPI1_BASEADDR
#define SPI2 					(SPI_RegDef_t*)SPI2_I2S2_BASEADDR
#define SPI3					(SPI_RegDef_t*)SPI3_I2S3_BASEADDR
#define SPI4					(SPI_RegDef_t*)SPI4_BASEADDR
#define SPI5					(SPI_RegDef_t*)SPI5_BASEADDR
#define SPI6					(SPI_RegDef_t*)SPI6_BASEADDR


/*********************TURN ON/OFF RCC FOR PERIPHRALS THAT USUAL USE**********************/
/*********************GPIO - I2C - SPI - USART - TIMER -  SYSCFG*************************/

//Turn on RCC for GPIO

#define GPIOA_PCLK_EN()			(pRCC->RCC_AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()			(pRCC->RCC_AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()			(pRCC->RCC_AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()			(pRCC->RCC_AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()			(pRCC->RCC_AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()			(pRCC->RCC_AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()			(pRCC->RCC_AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()			(pRCC->RCC_AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()			(pRCC->RCC_AHB1ENR |= (1<<8))
#define GPIOJ_PCLK_EN()			(pRCC->RCC_AHB1ENR |= (1<<9))
#define GPIOK_PCLK_EN()			(pRCC->RCC_AHB1ENR |= (1<<10))

//Turn on RCC for I2C

#define I2C1_PCLK_EN()			(pRCC->RCC_APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()			(pRCC->RCC_APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()			(pRCC->RCC_APB1ENR |= (1<<23))

//Turn on RCC for SPI

#define SPI1_PCLK_EN()			(pRCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			(pRCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			(pRCC->RCC_APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()			(pRCC->RCC_APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()			(pRCC->RCC_APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN()			(pRCC->RCC_APB2ENR |= (1 << 21))

// Turn on RCC for USART/UART

#define USART1_PCLK_EN()        (pRCC->RCC_APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()        (pRCC->RCC_APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()        (pRCC->RCC_APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()         (pRCC->RCC_APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()         (pRCC->RCC_APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()        (pRCC->RCC_APB2ENR |= (1 << 5))
#define UART7_PCLK_EN()         (pRCC->RCC_APB1ENR |= (1 << 30))
#define UART8_PCLK_EN()         (pRCC->RCC_APB1ENR |= (1 << 31))


// Turn on RCC for TIMERS (TIM1 to TIM11 in order)


#define TIM1_PCLK_EN()          (pRCC->RCC_APB2ENR |= (1 << 0))
#define TIM2_PCLK_EN()          (pRCC->RCC_APB1ENR |= (1 << 0))  // TIM2 is on APB1
#define TIM3_PCLK_EN()          (pRCC->RCC_APB1ENR |= (1 << 1))  // TIM3 is on APB1
#define TIM4_PCLK_EN()          (pRCC->RCC_APB1ENR |= (1 << 2))  // TIM4 is on APB1
#define TIM5_PCLK_EN()          (pRCC->RCC_APB1ENR |= (1 << 3))  // TIM5 is on APB1
#define TIM6_PCLK_EN()          (pRCC->RCC_APB1ENR |= (1 << 4))  // TIM6 is on APB1
#define TIM7_PCLK_EN()          (pRCC->RCC_APB1ENR |= (1 << 5))  // TIM7 is on APB1
#define TIM8_PCLK_EN()          (pRCC->RCC_APB2ENR |= (1 << 1))
#define TIM9_PCLK_EN()          (pRCC->RCC_APB2ENR |= (1 << 16))
#define TIM10_PCLK_EN()         (pRCC->RCC_APB2ENR |= (1 << 17))
#define TIM11_PCLK_EN()         (pRCC->RCC_APB2ENR |= (1 << 18))

// Turn on RCC for SYSCFG

#define SYSCFG_PCLK_EN()        (pRCC->RCC_APB2ENR |= (1 << 14))


//Turn off RCC for GPIO

#define GPIOA_PCLK_DI()			(pRCC->RCC_AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()			(pRCC->RCC_AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()			(pRCC->RCC_AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()			(pRCC->RCC_AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()			(pRCC->RCC_AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()			(pRCC->RCC_AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()			(pRCC->RCC_AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()			(pRCC->RCC_AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()			(pRCC->RCC_AHB1ENR &= ~(1<<8))
#define GPIOJ_PCLK_DI()			(pRCC->RCC_AHB1ENR &= ~(1<<9))
#define GPIOK_PCLK_DI()			(pRCC->RCC_AHB1ENR &= ~(1<<10))

// Turn off RCC for I2C

#define I2C1_PCLK_DI()          (pRCC->RCC_APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()          (pRCC->RCC_APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()          (pRCC->RCC_APB1ENR &= ~(1 << 23))


// Turn off RCC for SPI

#define SPI1_PCLK_DI()          (pRCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()          (pRCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()          (pRCC->RCC_APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()          (pRCC->RCC_APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()          (pRCC->RCC_APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI()          (pRCC->RCC_APB2ENR &= ~(1 << 21))

// Turn off RCC for USART/UART

#define USART1_PCLK_DI()        (pRCC->RCC_APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()        (pRCC->RCC_APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()        (pRCC->RCC_APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()         (pRCC->RCC_APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()         (pRCC->RCC_APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()        (pRCC->RCC_APB2ENR &= ~(1 << 5))
#define UART7_PCLK_DI()         (pRCC->RCC_APB1ENR &= ~(1 << 30))
#define UART8_PCLK_DI()         (pRCC->RCC_APB1ENR &= ~(1 << 31))

// Turn off RCC for TIMERS (TIM1 to TIM11 in order)

#define TIM1_PCLK_DI()          (pRCC->RCC_APB2ENR &= ~(1 << 0))
#define TIM2_PCLK_DI()          (pRCC->RCC_APB1ENR &= ~(1 << 0))
#define TIM3_PCLK_DI()          (pRCC->RCC_APB1ENR &= ~(1 << 1))
#define TIM4_PCLK_DI()          (pRCC->RCC_APB1ENR &= ~(1 << 2))
#define TIM5_PCLK_DI()          (pRCC->RCC_APB1ENR &= ~(1 << 3))
#define TIM6_PCLK_DI()          (pRCC->RCC_APB1ENR &= ~(1 << 4))
#define TIM7_PCLK_DI()          (pRCC->RCC_APB1ENR &= ~(1 << 5))
#define TIM8_PCLK_DI()          (pRCC->RCC_APB2ENR &= ~(1 << 1))
#define TIM9_PCLK_DI()          (pRCC->RCC_APB2ENR &= ~(1 << 16))
#define TIM10_PCLK_DI()         (pRCC->RCC_APB2ENR &= ~(1 << 17))
#define TIM11_PCLK_DI()         (pRCC->RCC_APB2ENR &= ~(1 << 18))


// Turn off RCC for SYSCFG

#define SYSCFG_PCLK_DI()        (pRCC->RCC_APB2ENR &= ~(1 << 14))


//Reset GPIOx Peripherals

#define GPIOA_REG_RESET()		do{(pRCC->RCC_AHB1RSTR |= (1<<0)); (pRCC->RCC_AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()		do{(pRCC->RCC_AHB1RSTR |= (1<<1)); (pRCC->RCC_AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()		do{(pRCC->RCC_AHB1RSTR |= (1<<2)); (pRCC->RCC_AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()		do{(pRCC->RCC_AHB1RSTR |= (1<<3)); (pRCC->RCC_AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()		do{(pRCC->RCC_AHB1RSTR |= (1<<4)); (pRCC->RCC_AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()		do{(pRCC->RCC_AHB1RSTR |= (1<<5)); (pRCC->RCC_AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()		do{(pRCC->RCC_AHB1RSTR |= (1<<6)); (pRCC->RCC_AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()		do{(pRCC->RCC_AHB1RSTR |= (1<<7)); (pRCC->RCC_AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()		do{(pRCC->RCC_AHB1RSTR |= (1<<8)); (pRCC->RCC_AHB1RSTR &= ~(1<<8));}while(0)
#define GPIOJ_REG_RESET()		do{(pRCC->RCC_AHB1RSTR |= (1<<9)); (pRCC->RCC_AHB1RSTR &= ~(1<<9));}while(0)
#define GPIOK_REG_RESET()		do{(pRCC->RCC_AHB1RSTR |= (1<<10)); (pRCC->RCC_AHB1RSTR &= ~(1<<10));}while(0)



//Reset SPIx Peripherals

#define SPI1_REG_RESET()		do{(pRCC->RCC_APB2RSTR |= (1<<12)); (pRCC->RCC_APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET()		do{(pRCC->RCC_APB1RSTR |= (1<<14)); (pRCC->RCC_APB1RSTR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET()		do{(pRCC->RCC_APB1RSTR |= (1<<15)); (pRCC->RCC_APB1RSTR &= ~(1<<15));}while(0)
#define SPI4_REG_RESET()		do{(pRCC->RCC_APB2RSTR |= (1<<13)); (pRCC->RCC_APB2RSTR &= ~(1<<13));}while(0)
#define SPI5_REG_RESET()		do{(pRCC->RCC_APB2RSTR |= (1<<20)); (pRCC->RCC_APB2RSTR &= ~(1<<20));}while(0)
#define SPI6_REG_RESET()		do{(pRCC->RCC_APB2RSTR |= (1<<21)); (pRCC->RCC_AHB1RSTR &= ~(1<<21));}while(0)
//#define IRQ Number trong NVIC
#define IRQ_NO_EXTI0 			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

// define SPI Number in NVIC
#define IRQ_NO_SPI1        35
#define IRQ_NO_SPI2        36
#define IRQ_NO_SPI3        51
#define IRQ_NO_SPI4        52
#define IRQ_NO_SPI5        53
#define IRQ_NO_SPI6        54

// Nghệ thuật do while để 1 define có thể làm được 2 việc
//Define some marco boolean

#define EN			1
#define DI			0
#define SET			1
#define RST			0
#define GPIO_PIN_SET	1
#define GPIO_PIN_RESET	0
#define FLAG_SET		1
#define FLAG_RESET		0
#define SPI_BUSY_FLAG 	(1<<7)
//Chuyển GPIO port người dùng nhập sang mã :
#define GPIO_BASEADDR_TO_CODE(x) 		   ((x == GPIOA)?0:\
											(x == GPIOB)?1:\
											(x == GPIOC)?2:\
											(x == GPIOD)?3:\
											(x == GPIOE)?4:\
											(x == GPIOF)?5:\
											(x == GPIOG)?6:\
											(x == GPIOH)?7:\
											(x == GPIOI)?8:\
											(x == GPIOJ)?9:0)
extern RCC_RegDef_t* pRCC;

#endif /* INC_STM32F429XX_H_ */
