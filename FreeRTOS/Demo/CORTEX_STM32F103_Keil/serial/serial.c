/*
    FreeRTOS V8.2.3 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
    BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0.
*/

/* Demo application includes. */
#include "serial.h"
/*-----------------------------------------------------------*/

/* Misc defines. */
#define serINVALID_QUEUE                ( ( QueueHandle_t ) 0 )
#define serNO_BLOCK                     ( ( TickType_t ) 0 )
#define serTX_BLOCK_TIME                ( 40 / portTICK_PERIOD_MS )

/*-----------------------------------------------------------*/

/* The queue used to hold received characters. */
//static QueueHandle_t xRxedChars;
//static QueueHandle_t xCharsForTx;
uart_rtos_handle_t uart1_handle;
uart_rtos_handle_t uart2_handle;
/*-----------------------------------------------------------*/

/* UART interrupt handler. */
void vUARTInterruptHandler( void );

/*-----------------------------------------------------------*/

/*
 * See the serial2.h header file.
 */
xComPortHandle xSerialPortInitMinimal(USART_TypeDef * base,uart_rtos_handle_t *handle, unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
{
    xComPortHandle xReturn;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Create the queues used to hold Rx/Tx characters. */
    handle->xRxedChars = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
    handle->xCharsForTx = xQueueCreate( uxQueueLength + 1, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
    handle->base = base;
    /* If the queue/semaphore was created correctly then setup the serial port
    hardware. */
    if( ( handle->xRxedChars != serINVALID_QUEUE ) && ( handle->xCharsForTx != serINVALID_QUEUE ) )
    {
        /* Enable USART1 clock */
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 | RCC_APB1Periph_USART2|RCC_APB2Periph_GPIOA, ENABLE );   

        /* Configure USART1 Rx (PA10) as input floating */
        (handle->base == USART1)? (GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10):(GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3);
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init( GPIOA, &GPIO_InitStructure );
        
        /* Configure USART1 Tx (PA9) as alternate function push-pull */
        (handle->base == USART1)? (GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9):(GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2);
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init( GPIOA, &GPIO_InitStructure );

        USART_InitStructure.USART_BaudRate = ulWantedBaud;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No ;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART_InitStructure.USART_Clock = USART_Clock_Disable;
        USART_InitStructure.USART_CPOL = USART_CPOL_Low;
        USART_InitStructure.USART_CPHA = USART_CPHA_2Edge;
        USART_InitStructure.USART_LastBit = USART_LastBit_Disable;
        
        USART_Init( handle->base, &USART_InitStructure );
        
        USART_ITConfig( handle->base, USART_IT_RXNE, ENABLE );
        
        if(handle->base == USART1)
        {
            NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
        }
        else
        {
            NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;
        }    
        
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init( &NVIC_InitStructure );
        
        USART_Cmd( handle->base, ENABLE );      
    }
    else
    {
        xReturn = ( xComPortHandle ) 0;
    }

    /* This demo file only supports a single port but we have to return
    something to comply with the standard demo header file. */
    return xReturn;
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialGetChar(uart_rtos_handle_t *handle,signed char *pcRxedChar, TickType_t xBlockTime )
{
    /* Get the next character from the buffer.  Return false if no characters
    are available, or arrive before xBlockTime expires. */
    if( xQueueReceive( handle->xRxedChars, pcRxedChar, xBlockTime ) )
    {
        return pdTRUE;
    }
    else
    {
        return pdFALSE;
    }
}
/*-----------------------------------------------------------*/

void vSerialPutString(uart_rtos_handle_t *handle, const signed char * const pcString, unsigned short usStringLength )
{
signed char *pxNext;

    /* A couple of parameters that this port does not use. */
    ( void ) usStringLength;
    /* NOTE: This implementation does not handle the queue being full as no
    block time is used! */

    /* Send each character in the string, one at a time. */
    pxNext = ( signed char * ) pcString;
    while( *pxNext )
    {
        xSerialPutChar(handle, *pxNext, serNO_BLOCK );
        pxNext++;
    }
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar(uart_rtos_handle_t *handle,signed char cOutChar, TickType_t xBlockTime )
{
signed portBASE_TYPE xReturn;

    if( xQueueSend( handle->xCharsForTx, &cOutChar, xBlockTime ) == pdPASS )
    {
        xReturn = pdPASS;
        USART_ITConfig( handle->base, USART_IT_TXE, ENABLE );
    }
    else
    {
        xReturn = pdFAIL;
    }

    return xReturn;
}
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
    /* Not supported as not required by the demo application. */
}
/*-----------------------------------------------------------*/

void vUARTInterruptHandler( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
char cChar;

    if( USART_GetITStatus(USART1, USART_IT_TXE ) == SET )
    {
        /* The interrupt was caused by the THR becoming empty.  Are there any
        more characters to transmit? */
        if( xQueueReceiveFromISR( uart1_handle.xCharsForTx, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
        {
            /* A character was retrieved from the queue so can be sent to the
            THR now. */
            USART_SendData(USART1, cChar );
        }
        else
        {
            USART_ITConfig( USART1, USART_IT_TXE, DISABLE );        
        }       
    }
    
    if( USART_GetITStatus(USART1, USART_IT_RXNE ) == SET )
    {
        cChar = USART_ReceiveData(USART1 );
        xQueueSendFromISR( uart1_handle.xRxedChars, &cChar, &xHigherPriorityTaskWoken );
    }   
    
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

void USART2_IRQHandler( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
char cChar;

    if( USART_GetITStatus(USART2, USART_IT_TXE ) == SET )
    {
        /* The interrupt was caused by the THR becoming empty.  Are there any
        more characters to transmit? */
        if( xQueueReceiveFromISR( uart2_handle.xCharsForTx, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
        {
            /* A character was retrieved from the queue so can be sent to the
            THR now. */
            USART_SendData(USART2, cChar );
        }
        else
        {
            USART_ITConfig( USART2, USART_IT_TXE, DISABLE );        
        }       
    }
    
    if( USART_GetITStatus(USART2, USART_IT_RXNE ) == SET )
    {
        cChar = USART_ReceiveData(USART2 );
        xQueueSendFromISR( uart2_handle.xRxedChars, &cChar, &xHigherPriorityTaskWoken );
    }   
    
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}





    
