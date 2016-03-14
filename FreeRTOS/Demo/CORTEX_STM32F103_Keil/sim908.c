#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "stm32f10x_lib.h"

/* Demo application includes. */
#include "serial.h"

/* Demo application includes. */
#include "sim908.h"

extern uart_rtos_handle_t uart2_handle;
SemaphoreHandle_t xMutex;
/*Get response from SIMCOM after send AT command*/
uint8_t GetResponse(char *buff_receive, uint32_t timeout)
{
    uint8_t count_char = 0;    
    TickType_t xtime;
    signed char SIM_RxChar;
    char cPassMessage[MAX_LENGH_STR];
    
    xtime = xTaskGetTickCount();
    
    do
    {        
        if (pdFALSE != xSerialGetChar(&uart2_handle, &SIM_RxChar, 10))
        {
            cPassMessage[count_char++] = SIM_RxChar;
        }
        else
        {
            if((SIM_RxChar == 0xA) && (cPassMessage[count_char -2] == 0xD) && (count_char >= 2))
                break;
        }      
    } while ((xTaskGetTickCount() - xtime < timeout )&&(count_char < MAX_LENGH_STR));

    if (count_char == MAX_LENGH_STR)
    {
        return pdFALSE;
    }
    cPassMessage[count_char] = '\0';
    strcpy(buff_receive, cPassMessage);
    
    return pdTRUE;
}

int8_t sendATcommand(char *ATcommand, char *expected_answer, unsigned int timeout)
{
    char buffer_response[MAX_LENGH_STR];
    // strcpy(buffdebug,ATcommand);
    if( xSemaphoreTake( xMutex, ( TickType_t ) portMAX_DELAY ) == pdTRUE )
    {
        printf("%s\r", ATcommand); // Send the AT command
        if (pdFALSE == GetResponse(buffer_response, timeout))
        {
            xSemaphoreGive( xMutex );
            return pdFALSE;
        }

        if (strstr(buffer_response, expected_answer) == NULL)
        {
            xSemaphoreGive( xMutex );
            return pdFALSE;
        }    
        xSemaphoreGive( xMutex );
    }

    return pdTRUE;
}

int8_t sendATcommand2(char *ATcommand, char *expected_answer,unsigned int timeout)
{
    uint8_t count_char = 0;    
    TickType_t xtime;
    signed char SIM_RxChar;
    char cPassMessage[MAX_LENGH_STR];
    
    if( xSemaphoreTake( xMutex, ( TickType_t ) portMAX_DELAY ) == pdTRUE )
    {
        printf("%s\r", ATcommand); // Send the AT command
        xtime = xTaskGetTickCount();
        
        do
        {        
            if (pdFALSE != xSerialGetChar(&uart2_handle, &SIM_RxChar,0xffff))
            {
                cPassMessage[count_char++] = SIM_RxChar;
            }
            if(strstr(cPassMessage,expected_answer))
            {
                break;
            }
        } while ((xTaskGetTickCount() - xtime < timeout )&&(count_char < MAX_LENGH_STR));

        if (count_char == MAX_LENGH_STR)
        {
            xSemaphoreGive( xMutex );
            return pdFALSE;
        }
        cPassMessage[count_char] = '\0';
        xSemaphoreGive( xMutex );        
    }
    return pdTRUE;
}

uint8_t GPS_PWR()
{
   // Power up GPS
     sendATcommand("AT+CGPSPWR=1", "OK", 2000);    
    // Reset GPS Hot m0de
     sendATcommand("AT+CGPSRST=1", "OK", 2000);
     return pdTRUE;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : start_GPS
 * Description   : Starting GPS for modul SIM908
 * This function
 *
 *END**************************************************************************/
BaseType_t Wait_GPS_Fix(void)
{
    // waits for fix GPS
    if ((pdTRUE == sendATcommand("AT+CGPSSTATUS?", "Location 2D Fix", 2000)) ||
        (pdTRUE == sendATcommand("AT+CGPSSTATUS?", "Location 3D Fix", 2000)))
    {
        return pdTRUE;
    }
    else
    {
        return pdFALSE;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : get_GPS
 * Description   : get_GPS GPRS for modul SIM908
 * This function use config get parameter GPS
 *
 *END**************************************************************************/
uint8_t get_GPS(GPS_INFO *vGPSinfo)
{   
    char buffer_response[MAX_LENGH_STR];
    //GPS_INFO vGPSinfo;
    // First get the NMEA string
    printf("AT+CGPSINF=0\r");
    if(pdTRUE == GetResponse(buffer_response,2000))
    {
        strtok(buffer_response, ",");
        strcpy(vGPSinfo->longtitude,strtok(NULL, ",")); // Gets longitude
        strcpy(vGPSinfo->latitude,strtok(NULL, ",")); // Gets latitude
        //strcpy(altitude,strtok(NULL, ".")); // Gets altitude
        strtok(NULL, ".");
        strtok(NULL, ",");
        strcpy(vGPSinfo->date,strtok(NULL, ".")); // Gets date
        strtok(NULL, ",");
        //free(buffer_gps_t) ;
        return pdTRUE ;
    }
    return pdFALSE;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : Config_GPRS_SIM908
 * Description   : Config GPRS for modul SIM908
 * This function use config gprs to sim908
 *
 *END**************************************************************************/
void Config_GPRS_SIM908(void)
{
    //    sendATcommand("AT+CIPCSGP=1,\"v-internet\",\"\",\"\"","OK", 2000); // For Viettel Network
    // VinaPhone
    sendATcommand("AT+CIPCSGP=1,\"3m-world\",\"mms\",\"mms\"", "OK", 2000); // For Vina Network
}

/*FUNCTION**********************************************************************
 *
 * Function Name : Tcp_Connect
 * Description   :
 * This function
 *AT+CIPSTART="TCP","42.115.190.28","8888"
 *END**************************************************************************/
TCP_STATUS TCP_Connect(char *IP_address, char *Port)
{
    char command[70];
    memset(command, '\0', 70);
    sendATcommand("AT+CIPSHUT", "SHUT OK", 5000);
    // delay(2000) ;
    sprintf(command, "AT+CIPSTART=\"TCP\",\"%s\",\"%s\"", IP_address, Port);
    if (pdTRUE == sendATcommand2(command,"CONNECT OK", 60000))
    {
        return TCP_SUCCESS;
    }
    return TCP_CONNECT_FAIL;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : TCP_Send
 * Description   : `
 * This function
 *
 *END**************************************************************************/
TCP_STATUS TCP_Send(char *data_string)
{
    // char data_ctrl_z[120];
    char *data_ctrl_z;
    TCP_STATUS status;
    data_ctrl_z = malloc(strlen(data_string) + 2);
    if (1 == sendATcommand2("AT+CIPSTATUS", "CONNECT OK", 20000))
    {
        // memset(data_ctrl_z , '\0',120);
        if (sendATcommand2("AT+CIPSEND", ">", 20000))
        {
            sprintf(data_ctrl_z, "%s%c", data_string, 26);
            if (!sendATcommand2(data_ctrl_z, "SEND OK", 30000))
            {
                status = TCP_SEND_TIMEOUT;
            }
            else
            {
                status = TCP_SUCCESS;
            }
        }
        else
        {
            status = TCP_FAIL;
        }
    }
    else // if(1 == sendATcommand("AT+CIPSTATUS","TCP CLOSE",10000))
    {
        // free(data_ctrl_z);
        status = TCP_CONNECT_FAIL;
    }
    free(data_ctrl_z);
    return status;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : TCP_Close
 * Description   :
 * This function
 *
 *END**************************************************************************/
TCP_STATUS TCP_Close(void)
{
    // Closes the socket
    if (pdTRUE == sendATcommand("AT+CIPCLOSE", "CLOSE OK", 10000))
    {
        return TCP_SUCCESS;
    }
    return TCP_FAIL;
}

uint8_t GetAccount()
{
    char SIM_RxChar;
    char buffer_acc[160] , *ptr_buff;
    uint16_t cnt = 0;
    ptr_buff = buffer_acc;

    if (sendATcommand("AT+CUSD=1,\"*101#\"", "OK", 2000))
    {
        do {
            if (pdFALSE != xSerialGetChar(&uart2_handle, &SIM_RxChar, 0xffff))
            {
                *(ptr_buff++) = SIM_RxChar;            
                cnt++;
            }
            if((*(ptr_buff - 2)==0xD) && (SIM_RxChar ==0xA) && (cnt > 2))
            {
                break;
            }
        }while(1) ;
    }
    *ptr_buff = '\0';
    return pdTRUE;
}
void Sim908_setup(void)
{

    xMutex = xSemaphoreCreateMutex();
    if( xMutex == NULL )
    {
        while(1);
    }    
    Sim908_power_on(); // Power up Sim908 module
    /*****Config Sim908 Module *****************************/
    sendATcommand("ATE0", "OK", 2000);            // off echo
    sendATcommand("AT+CFUN=1", "OK", 2000);       // off echo
    sendATcommand("AT+CIPSHUT", "SHUT OK", 3000); // disconect gprs
    sendATcommand("AT+CSCLK=1", "OK", 2000);      // sleep mode
    sendATcommand("AT+CMGF=1", "OK", 2000);
    // GPIO_WriteLow(DTR_GPIO_PORT, (GPIO_Pin_TypeDef)DTR_GPIO_PINS); //wake up
    // Power up GPS
    sendATcommand("AT+CGPSPWR=1", "OK", 2000); // power up gps
    // Reset GPS Cold mde
    sendATcommand("AT+CGPSRST=1", "OK", 2000);
    sendATcommand("AT+CREG=2", "OK", 2000);
    /************End Config Sim908 Module *****************************/
    // delay(1000);
    while (sendATcommand("AT+CREG?", "+CREG: 2,1", 2000) == pdFALSE);
    // Configure DNS server address
    sendATcommand("AT+CGATT", "OK", 2000);
    // delay(1000);
    sendATcommand("AT+CSTT=\"3m-world\",\"mms\",\"mms\"", "OK", 2000);
    sendATcommand("AT+CIICR", "OK", 8000);
    // delay(5000);
    sendATcommand("AT+CIPSTATUS", "OK", 3000);
    // delay(2000);
    sendATcommand("AT+CIFSR", "OK", 3000);
    // delay(4000);
    sendATcommand("AT+CDNSCFG=\"8.8.8.8\",\"4.4.4.4\"", "OK", 2000);
    // delay(2000);
    sendATcommand("AT&W", "OK", 2000);
    /*Config for first time*/
    // Config_GPRS_SIM908();
}

void Sim908_power_on(void)
{

    if (pdFALSE == sendATcommand("AT", "OK", 2000))
    { // power on pulse
        SIM908_PWRON;
        delay_ms(3000);
        SIM908_PWROFF;
        // Wake up
        // waits for an answer from the module
        printf("ATE0\r");
        while(pdFALSE == sendATcommand("AT", "OK", 2000));
    }

    printf("ATE0\r");
}
