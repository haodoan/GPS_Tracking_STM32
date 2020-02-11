/**
 ******************************************************************************
 * @file    stm32_spi.c
 * @author  Alexei Troussov
 * @version V1.0
 * @date    21-June-2012
 * @brief   This file contains definitions and routines to handle SPI bus
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "stm32_spi.h"
#include "stm32_pins.h"
/** @addtogroup Utilities
 * @{
 */

/** @defgroup STM32_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 *//* STM32_Private_TypesDefinitions */


/** @defgroup STM32_Private_Defines
 * @{
 */
/**
 * @}
 *//* STM32_Private_Defines */


/** @defgroup STM32_Private_Macros
 * @{
 */
/**
 * @}
 *//* STM32_Private_Macros */


/** @defgroup STM32_Private_Constants
 * @{
 */
/**
 * @}
 *//* STM32_Private_Constants */


/** @defgroup STM32_Private_Variables
 * @{
 */
/**
 * @}
 *//* STM32_Private_Variables */


/** @defgroup STM32_Private_FunctionPrototypes
 * @{
 */
/**
 * @}
 *//* STM32_Private_FunctionPrototypes */


/** @defgroup STM32_Public_Functions
 * @{
 */

/**
 * @brief  Initialize SPI
 * @param  None
 * @retval None
 */
#if 1
void STM_EVAL_SPI_Init( void )
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/*!< SD_SPI_CS_GPIO, SD_SPI_MOSI_GPIO, SD_SPI_MISO_GPIO, SD_SPI_DETECT_GPIO 
       and SD_SPI_SCK_GPIO Periph clock enable */
  RCC_APB2PeriphClockCmd(SD_CS_GPIO_CLK | SD_SPI_MOSI_GPIO_CLK | SD_SPI_MISO_GPIO_CLK |
                         SD_SPI_SCK_GPIO_CLK , ENABLE);

  /*!< SD_SPI Periph clock enable */
  RCC_APB2PeriphClockCmd(SD_SPI_CLK, ENABLE); 

  
  /*!< Configure SD_SPI pins: SCK */
  GPIO_InitStructure.GPIO_Pin = SD_SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SD_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure SD_SPI pins: MOSI */
  GPIO_InitStructure.GPIO_Pin = SD_SPI_MOSI_PIN;
  GPIO_Init(SD_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure SD_SPI pins: MISO */
  GPIO_InitStructure.GPIO_Pin = SD_SPI_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_Init(SD_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
  

	/* SPI configuration -------------------------------------------------------*/
	SPI_DeInit( SPIx_SPI );

	/* Initializes the SPI communication */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 10;
  SPI_Init(SD_SPI, &SPI_InitStructure);

	/* The Data transfer is performed in the SPI interrupt routine */
	SPI_Cmd( SPIx_SPI, ENABLE );  /* Enable the SPI peripheral */
}
#endif
/**
 * @brief  Sends a byte on SPI bus and receives a byte of response
 * @see SD_WriteByte() and SD_ReadByte() from stm32_eval_spi_sd.c
 * @param  Byte to send
 * @retval Received data
 */
uint16_t STM_EVAL_SPI_Send_Recieve_Data( uint8_t data )
{
	/* Wait until the transmit buffer is empty */
	while ( SPI_GetFlagStatus( SPIx_SPI, SPI_FLAG_TXE ) == RESET ) {}
	SPI_SendData( SPIx_SPI, data );	/* Send byte to SPI bus */
	/* Wait to receive a byte */
	while ( SPI_GetFlagStatus( SPIx_SPI, SPI_FLAG_RXNE ) == RESET ) {}
	return SPI_ReceiveData( SPIx_SPI );	/* Read byte from SPI bus */
}

//void STM_EVAL_SPI_Low_Speed()
//{
//	SPI_InitTypeDef SPI_InitStructure;
//
//	SPI_Cmd( SPIx_SPI, DISABLE );
//	SPI_I2S_DeInit( SPIx_SPI );
//
//	/* Initializes the SPI communication */
//	SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
//	SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
//	SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;
//	SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;
//	SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
//	SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
//	SPI_InitStructure.SPI_CRCPolynomial     = 7;
//	SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
//	SPI_Init( SPIx_SPI, &SPI_InitStructure );
//
//	/* The Data transfer is performed in the SPI interrupt routine */
//	SPI_Cmd( SPIx_SPI, ENABLE );	/* Enable the SPI peripheral */
//}
//
//void STM_EVAL_SPI_High_Speed()
//{
//	SPI_InitTypeDef SPI_InitStructure;
//
//	SPI_Cmd( SPIx_SPI, DISABLE );
//	SPI_I2S_DeInit( SPIx_SPI );
//
//	/* Initializes the SPI communication */
//	SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
//	SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
//	SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;
//	SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;
//	SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
//	SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
//	SPI_InitStructure.SPI_CRCPolynomial     = 7;
//	SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
//	SPI_Init( SPIx_SPI, &SPI_InitStructure );
//
//	/* The Data transfer is performed in the SPI interrupt routine */
//	SPI_Cmd( SPIx_SPI, ENABLE );	/* Enable the SPI peripheral */
//}

/**
 * @}
 *//* STM32_Public_Functions */


/**
 * @}
 *//* Utilities */
