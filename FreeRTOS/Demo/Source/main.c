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
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks.
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Fast Interrupt Test" - A high frequency periodic interrupt is generated
 * using a free running timer to demonstrate the use of the
 * configKERNEL_INTERRUPT_PRIORITY configuration constant.  The interrupt
 * service routine measures the number of processor clocks that occur between
 * each interrupt - and in so doing measures the jitter in the interrupt timing.
 * The maximum measured jitter time is latched in the ulMaxJitter variable, and
 * displayed on the LCD by the 'Check' task as described below.  The
 * fast interrupt is configured and handled in the timertest.c source file.
 *
 * "LCD" task - the LCD task is a 'gatekeeper' task.  It is the only task that
 * is permitted to access the display directly.  Other tasks wishing to write a
 * message to the LCD send the message on a queue to the LCD task instead of
 * accessing the LCD themselves.  The LCD task just blocks on the queue waiting
 * for messages - waking and displaying the messages as they arrive.
 *
 * "Check" task -  This only executes every five seconds but has the highest
 * priority so is guaranteed to get processor time.  Its main function is to
 * check that all the standard demo tasks are still operational.  Should any
 * unexpected behaviour within a demo task be discovered the 'check' task will
 * write an error to the LCD (via the LCD task).  If all the demo tasks are
 * executing with their expected behaviour then the check task writes PASS
 * along with the max jitter time to the LCD (again via the LCD task), as
 ":.k, * described above.
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <string.h>
/* Scheduler includes. */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#include "stm32f10x_it.h"

/* Demo app includes. */
#include "serial.h"
#include "sim908.h"
#include "lcd_nokia5110/nokia_5110.h"

#include "FAT/ff.h"
#include "stm32_sd_spi.h"
#include "stm32_spi.h"



/* Task priorities. */
#define mainGPS_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define mainGPRS_TASK_PRIORITY (tskIDLE_PRIORITY + 4)
#define mainSAVE_LOCATION_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define mainLED_TASK_PRIORITY (tskIDLE_PRIORITY)
/* The check task uses the sprintf function so requires a little more stack. */
#define mainGPS_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 256)
#define mainGPRS_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 2048)
#define mainLED_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define mainSAVELOC_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 2048)
/* The time between cycles of the 'check' task. */
#define mainGPS_DELAY ((TickType_t)5000 / portTICK_PERIOD_MS)
#define mainLED_BLINK_DELAY (500)

#define GPRS_BLOCK_TIME 5000
#define SAVELOC_BLOCK_TIME 5000
#define GPRS_BUFFER_SIZE 320

#define GPS_BLOCK_TIME 5000

#define SERVER          "http://store.redboxsa.com/update-location"
#define SERVER_OFFLINE  "http://store.redboxsa.com/update-location-offline"
// #define SERVER          "http://ptsv2.com/t/GPSonlineSrv/post"
//#define SERVER_OFFLINE  "http://ena1kgc4bexxh.x.pipedream.net/"
//#define TESTING  1
#define RESPONSE_DATA   "\"state\":true"
/*-----------------------------------------------------------*/

/*
 * Configure the clocks, GPIO and other peripherals as required by the demo.
 */
static void prvSetupHardware(void);

/*
 * The LCD is written two by more than one task so is controlled by a
 * 'gatekeeper' task.  This is the only task that is actually permitted to
 * access the LCD directly.  Other tasks wanting to display a message send
 * the message to the gatekeeper.
 */
/*
 * Retargets the C library printf function to the USART.
 */
int fputc(int ch, FILE *f);

/*
 * Checks the status of all the demo tasks then prints a message to the
 * display.  The message will be either PASS - and include in brackets the
 * maximum measured jitter time (as described at the to of the file), or a
 * message that describes which of the standard demo tasks an error has been
 * discovered in.
 *
 * Messages are not written directly to the terminal, but passed to vLCDTask
 * via a queue.
 */
static void vGPSTask(void *pvParameters);

static void vGPRSTask(void *pvParameters);
static uint32_t WriteGPSDataInfo(uint32_t sector, GPS_INFO gpsInfo);
static void ReadGPSInfo(uint32_t sector, char *gpsBuff);
static uint32_t WriteJsonHeadertoSDcard(GPS_INFO gpsInfo);
/*
 * Configures the timers and interrupts for the fast interrupt test as
 * described at the top of this file.
 */
extern void vSetupTimerTest(void);
void error_lcd_printf(void);

/*handler for using USART*/
extern uart_rtos_handle_t uart1_handle;
extern uart_rtos_handle_t uart2_handle;

QueueHandle_t SIM908_queue;
SemaphoreHandle_t SIM908_Mutex;
/*-----------------------------------------------------------*/

int main(void)
{
#ifdef DEBUG
    debug();
#endif

    prvSetupHardware();
    
    /* Start the standard demo tasks. */
    //  vStartBlockingQueueTasks( mainBLOCK_Q_PRIORITY );
    //  vCreateBlockTimeTasks();
    //  vStartSemaphoreTasks( mainSEM_TEST_PRIORITY );
    //  vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
    //  vStartIntegerMathTasks( mainINTEGER_TASK_PRIORITY );
    //vStartLEDFlashTasks( mainLED_TASK_PRIORITY );
    //  vAltStartComTestTasks( mainCOM_TEST_PRIORITY, mainCOM_TEST_BAUD_RATE, mainCOM_TEST_LED );
    
    
    SIM908_queue = xQueueCreate(20, sizeof(GPS_INFO));
    if (SIM908_queue == NULL)
    {
        //LCD_write_string(0, 3, "QUEUE MEM fAIL");
        while (TRUE)
            ;
    }
    SIM908_Mutex = xSemaphoreCreateMutex();
    if( SIM908_Mutex == NULL )
    {
        //LCD_write_string(0, 3, "MUTEX MEM fAIL");
        while(1);
    }

    /* Start the tasks defined within this file/specific to this demo. */
    // xTaskCreate( vGPSTask, "GPS", mainGPS_TASK_STACK_SIZE, NULL, mainGPS_TASK_PRIORITY, NULL );
    xTaskCreate(vGPRSTask, "GPRS", mainGPRS_TASK_STACK_SIZE, NULL, mainGPRS_TASK_PRIORITY, NULL);

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* Will only get here if there was not enough heap space to create the
    idle task. */
    return 0;
}
/*-----------------------------------------------------------*/
/*GPS task*/
static void vGPSTask(void *pvParameters)
{
    GPS_INFO vGPSinfo;

    /*GPS Power ON*/
    GPS_PWR();
    /*get imei number of module*/
    GetIMEI(vGPSinfo.IMEI);

    vGPSinfo.MCC = 452;
    vGPSinfo.MNC = 2; //Vinaphone

#ifdef TESTING// for testing
    strcpy(vGPSinfo.latitude,  "2101.546917");
    strcpy(vGPSinfo.longtitude,"10547.778668");
    strcpy(vGPSinfo.IMEI,"861001005868241");
    strcpy(vGPSinfo.date,"20200313200159");
#endif
    for (;;)
    {
        if( xSemaphoreTake( SIM908_Mutex, ( TickType_t ) portMAX_DELAY ) == pdTRUE )
        {

#ifdef TESTING // for testing
            vGPSinfo.FIX = pdTRUE;
#else
            if (pdTRUE == Wait_GPS_Fix())
            {
                //LCD_write_string(0, 0, "Fix        ");
                SIM908_LED_GPSFIX_ON ;
                vGPSinfo.FIX  = pdTRUE;
                //memset(&vGPSinfo, '\0', sizeof(GPS_INFO));
                get_GPS(&vGPSinfo);
            }
            else // get cell id
            {
               // LCD_write_string(0, 0, "Not Fix");
                SIM908_LED_GPSFIX_OFF;
                vGPSinfo.FIX = pdFALSE;
                //memset(&vGPSinfo, '\0', sizeof(GPS_INFO));
                //GetCellid(&vGPSinfo);
            }
#endif

            if(SIM908_RF_DISABLE_INPUT == Bit_RESET)
            {
                Disable_RF();
            }
            else
            {
                Enable_RF();
            }
            xSemaphoreGive( SIM908_Mutex );
        }
        if (xQueueSend(SIM908_queue, &vGPSinfo, GPS_BLOCK_TIME) != pdPASS)
        {
            //LCD_write_string(0, 3, "Queue fully   ");
        }
        vTaskDelay(5000);
    }
}
/*GPRS task*/
static void vGPRSTask(void *pvParameters)
{
    GPS_INFO vGPSinfo;
    HTTP_STATUS httpStatus = HTTP_POST_SUCCESS;
    char gprs_buffer[GPRS_BUFFER_SIZE] ;
    uint32_t sector = 1 ;
    uint32_t size = 0;
    uint32_t latestOnlineStatus = pdTRUE;
    uint32_t index;
    uint32_t tmpSize;
    uint32_t cntError;
    // Set up sim908
    Sim908_setup();
    // setting gprs for Sim module
    Config_GPRS_SIM908();

    HTTP_Init(SERVER);

    STM_EVAL_SPI_Init();
    SD_Init();   


#ifdef TESTING// for testing
    strcpy(vGPSinfo.latitude,  "2101.546917");
    strcpy(vGPSinfo.longtitude,"10547.778668");
    strcpy(vGPSinfo.IMEI,"861001005868241");
    strcpy(vGPSinfo.date,"20200313200159");
    vGPSinfo.FIX = pdTRUE;
    

    
    for(index = 0; index <  2500; index++)
    {
        size += WriteGPSDataInfo(sector, vGPSinfo);
        sector++ ;
    }    
/*    HTTP_Init(SERVER_OFFLINE);
    size += WriteJsonHeadertoSDcard(vGPSinfo);
    HTTP_POST_BIGSIZE_FromSD(sector, size , 30000, ReadGPSInfo) ;*/
    
#endif

    xTaskCreate(vGPSTask, "GPS", mainGPS_TASK_STACK_SIZE, NULL, mainGPS_TASK_PRIORITY, NULL);
    
    for (;;)
    {
        if (xQueueReceive(SIM908_queue, &vGPSinfo, GPRS_BLOCK_TIME)) // Receive data from GPS task
        {
            if (vGPSinfo.FIX == pdFALSE) // GPS not fix
            {
/*                memset(gprs_buffer, '\0', sizeof(gprs_buffer) / sizeof(char));
                sprintf(gprs_buffer, "%s,%s,0,0,%d,%d,%s,%s,0,0,0,%s\r\n", GPRS_HEAD_CMD, vGPSinfo.IMEI, vGPSinfo.MCC,
                    vGPSinfo.MNC,vGPSinfo.LAC,vGPSinfo.CELLID, GPRS_END_CMD);*/
            }
            else // GPS 
            {
                if( xSemaphoreTake( SIM908_Mutex, ( TickType_t ) portMAX_DELAY ) == pdTRUE )
                {
                    memset(gprs_buffer, '\0', sizeof(gprs_buffer) / sizeof(char));
                    jsonDataPost(vGPSinfo,gprs_buffer);

                    if(CheckIPaddressExist())
                    {
                        if (HTTP_POST_SUCCESS == HTTP_Post(gprs_buffer , 10000))
                        {                                  
                            if(HTTP_READ_SUCCESS == HTTP_Read(RESPONSE_DATA))
                            {
                                vGPSinfo.ONLINE = pdTRUE ;
                                SIM908_LED_GPSFIX_OFF;
                                if(latestOnlineStatus == pdFALSE)
                                {
                                    size += WriteJsonHeadertoSDcard(vGPSinfo);
                                    cntError = 0;
                                    do
                                    {
                                        vTaskDelay(500);
                                        HTTP_Init(SERVER_OFFLINE);
                                        httpStatus = HTTP_POST_BIGSIZE_FromSD(sector, size , 20000, ReadGPSInfo) ;
                                        if(++cntError == 1) break;
                                    }while( httpStatus != HTTP_POST_SUCCESS );
                                   
                                    vTaskDelay(500);
                                    SIM908_LED_GPSFIX_OFF;
                                    HTTP_Init(SERVER);
                                    #ifndef TESTING
                                    sector = 1; // sector start for write data  
                                    size = 0;
                                    #endif                      
                                }
                                latestOnlineStatus = pdTRUE;                     
                            }
                        }
                        else
                        {
                            #ifndef TESTING
                            vGPSinfo.ONLINE = pdFALSE ;
                            latestOnlineStatus = pdFALSE;
                            vTaskDelay(10000);
                            tmpSize =  WriteGPSDataInfo(sector, vGPSinfo);
                            size += tmpSize;
                            if(tmpSize != 0)
                            {
                                sector++ ;                                
                            }

                            HTTP_Init(SERVER);
                            vTaskDelay(1000);
                            #else 
                            vTaskDelay(500);
                            HTTP_Init(SERVER);
                            vTaskDelay(500);
                            #endif        
                        }                          
                    }
                    else
                    {
                        vGPSinfo.ONLINE = pdFALSE ;
                        latestOnlineStatus = pdFALSE;
                    #ifndef TESTING
                        vTaskDelay(10000);
                        tmpSize =  WriteGPSDataInfo(sector, vGPSinfo);
                        size += tmpSize;
                        if(tmpSize != 0)
                        {
                            sector++ ;                                
                        }
                        HTTP_Init(SERVER);
                        vTaskDelay(1000);
                    #else 
                        vTaskDelay(500);
                        HTTP_Init(SERVER);
                        vTaskDelay(500);  
                    #endif        
                    }       
                    xSemaphoreGive( SIM908_Mutex );
                }
            
            }

        }  
    }
}

static uint32_t WriteJsonHeadertoSDcard(GPS_INFO gpsInfo)
{
    char buff[256] ;
        uint32_t size;

    sprintf(buff,"{\
\"id\":%s,\
\"online\":false,\
\"gps\":true,\
\"position\":[\n",\
gpsInfo.IMEI);    

    SD_SectorWrite(0, (uint8_t *)buff);
    size = strlen(buff);

    return size ;

}        
static uint32_t WriteGPSDataInfo(uint32_t sector, GPS_INFO gpsInfo)
{

    uint8_t buff[256] ;
    uint32_t size;
    static uint32_t referenceSize;

    sprintf((char*)buff, "{\"date\":%s,\"lat\":%s,\"lng\":%s,\"speed\":%d,\"bearing\":%d,\"fuel\":%d}", gpsInfo.date,gpsInfo.latitude \
                                                                                          , gpsInfo.longtitude,10,20,0) ;
    if(sector == 1)
    {
        referenceSize = strlen((char*)buff);        
    }
    size =  strlen((char*)buff);
    if(referenceSize == size)
    {
        SD_SectorWrite(sector, buff);      
    }
		else
		{
			size = 0;
		}

    return size ;
}

static void ReadGPSInfo(uint32_t sector, char *gpsBuff)
{

    SD_SectorRead( sector, (uint8_t *)gpsBuff );
}


/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable HSE (high speed external clock). */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready. */
    while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
    {
    }

    /* 2 wait states required on the flash. */
    *((unsigned long *)0x40022000) = 0x02;

    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 12MHz * 6 = 72 MHz. */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);

    /* Enable PLL. */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready. */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source. */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source. */
    while (RCC_GetSYSCLKSource() != 0x08)
    {
    }

    /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO,
                           ENABLE);

    /* SPI2 Periph clock enable */
    // RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );

    /* Set the Vector Table base address at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* Configure HCLK clock as SysTick clock source. */
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

    xSerialPortInitMinimal(USART2, &uart2_handle, 115200, MAX_LENGH_STR);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*Initial for led GPS fix*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*Initial input disale RF*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*Initial for PWKEY*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // vParTestInitialise();
}
/*-----------------------------------------------------------*/

void error_lcd_printf()
{
    LCD_write_string(0, 3, "FAIL...");
    while (TRUE)
        ;
}
int fputc(int ch, FILE *f)
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    xSerialPutChar(&uart2_handle, ch, 0);

    return ch;
}
/*-----------------------------------------------------------*/

#ifdef DEBUG
/* Keep the linker happy. */
void assert_failed(unsigned char *pcFile, unsigned long ulLine)
{
    for (;;)
    {
    }
}
#endif
