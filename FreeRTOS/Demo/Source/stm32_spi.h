/**
 ******************************************************************************
 * @file    stm32_spi.h
 * @author  Alexei Troussov
 * @version V1.0
 * @date    21-June-2012
 * @brief   This file contains definitions and routines to handle SPI bus
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32_SPI_H
#define STM32_SPI_H


/* Includes ------------------------------------------------------------------*/
//#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f10x_lib.h"

/** @addtogroup Utilities
 * @{
 */

/** @defgroup STM32_Exported_Types
 * @{
 */
/**
 * @}
 *//* STM32_Exported_Types */

/** @defgroup STM32_Exported_Constants
 * @{
 */
/**
 * @}
 *//* STM32_Exported_Constants */

/** @defgroup STM32_Exported_Functions
 * @{
 */

void STM_EVAL_SPI_Init( void );
uint16_t STM_EVAL_SPI_Send_Recieve_Data( uint8_t data );
//void STM_EVAL_SPI_High_Speed( void );
//void STM_EVAL_SPI_Low_Speed( void );

/**
 * @}
 *//* STM32_Exported_Functions */

/**
 * @}
 *//* Utilities */

#endif /* STM32_SPI_H */