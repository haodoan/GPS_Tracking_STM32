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
        if (pdFALSE != xSerialGetChar(&uart2_handle, &SIM_RxChar, 20))
        {
            cPassMessage[count_char++] = SIM_RxChar;
        }
        else
        {
            if((SIM_RxChar == 0xA) && (cPassMessage[count_char -2] == 0xD) && (count_char > 4))
            {
                break;
            }
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

int8_t SendATcommand2(char *ATcommand,char *expected_answer,char *expected_answer2, unsigned int timeout)
{
    char buffer_response[MAX_LENGH_STR];
    signed char SIM_RxChar;
    uint32_t error = pdTRUE;

    if( xSemaphoreTake( xMutex, ( TickType_t ) portMAX_DELAY ) == pdTRUE )
    {
        while(pdTRUE == xSerialGetChar(&uart2_handle, &SIM_RxChar,100));
        printf("%s\r", ATcommand); // Send the AT command
        if (pdFALSE == GetResponse(buffer_response, timeout))
        {
            error = pdFALSE;
        }
        else
        {
            if (strstr(buffer_response, expected_answer))
            {
                GetResponse(buffer_response, timeout);
                if (strstr(buffer_response, expected_answer2) == NULL)
                {
                    error = pdFALSE;
                }
            }        
        }
        xSemaphoreGive( xMutex );        
    }
       
    return error;
}

int8_t SendATcommand(char *ATcommand, char *expected_answer,unsigned int timeout)
{
    uint8_t count_char = 0;
    uint8_t error = pdTRUE;
    signed char SIM_RxChar;
    char cPassMessage[MAX_LENGH_STR];

    memset(cPassMessage,'\0',MAX_LENGH_STR);
    if( xSemaphoreTake( xMutex, ( TickType_t ) portMAX_DELAY ) == pdTRUE )
    {
        while(pdTRUE == xSerialGetChar(&uart2_handle, &SIM_RxChar,200));
        printf("%s\r", ATcommand); // Send the AT command
        do
        {
            if (pdFALSE != xSerialGetChar(&uart2_handle, &SIM_RxChar,timeout))
            {
                cPassMessage[count_char++] = SIM_RxChar;
            }
            else
            {
                error = pdFAIL;
                break;
            }
            if(strstr(cPassMessage,expected_answer))
            {
                break;
            }
        } while (count_char < MAX_LENGH_STR);

        if ((count_char == MAX_LENGH_STR) || (error == pdFAIL))
        {
            xSemaphoreGive( xMutex );
            return pdFALSE;
        }
        cPassMessage[count_char] = '\0';
        xSemaphoreGive( xMutex );
				
		return pdTRUE;
    }
		else
			return pdFALSE;
}

uint8_t GPS_PWR()
{
   // Power up GPS
     SendATcommand("AT+CGPSPWR=1", "OK", 2000);
    // Reset GPS Hot bmode
     SendATcommand("AT+CGPSRST=1", "OK", 2000);
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
    if(pdTRUE == SendATcommand("AT+CGPSSTATUS?", "3D Fix", 2000))
    {
        return pdTRUE;
    }
    else
    {
        return(SendATcommand("AT+CGPSSTATUS?", "2D Fix", 2000));
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
    uint32_t error;
    signed char SIM_RxChar;
    if( xSemaphoreTake( xMutex, ( TickType_t ) portMAX_DELAY ) == pdTRUE )
    { 
        while(pdTRUE == xSerialGetChar(&uart2_handle, &SIM_RxChar,100));
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
            error = pdTRUE ;
        }
        else {error = pdFAIL;}
        
        xSemaphoreGive( xMutex );
    }
    return error;
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

    /* For TCPIP GPRS*/
    //    SendATcommand("AT+CIPCSGP=1,\"v-internet\",\"\",\"\"","OK", 2000); // For Viettel Network
    // VinaPhone
    //SendATcommand("AT+CIPCSGP=1,\"3m-world\",\"mms\",\"mms\"", "OK", 2000); // For Vina Network

    /*for HTTP GPRS */
    SendATcommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"","OK",2000);
    SendATcommand("AT+SAPBR=3,1,\"APN\",\"v-internet\"","OK",2000);
    SendATcommand("AT+SAPBR=1,1","OK",2000);
    SendATcommand("AT+SAPBR=2,1","OK",2000);
	
}

/*FUNCTION**********************************************************************
 *
 * Function Name : Tcp_Connect
 * Description   :
 * This function
 *AT+CIPSTART="TCP","42.115.190.28","8888"
 *END**************************************************************************/
TCP_STATUS TCP_Connect(char *IP_address, char *Port,uint32_t timeout)
{
    char command[50];

    memset(command, '\0', 50);
    SendATcommand("AT+CIPSHUT", "SHUT OK", 5000);
    sprintf(command, "AT+CIPSTART=\"TCP\",\"%s\",\"%s\"", IP_address, Port);
    if (pdTRUE == SendATcommand2(command,"OK","CONNECT OK", timeout))
    {
        return TCP_CONNECT_SUCCESS;
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
    //char data_ctrl_z[120];
    char *data_ctrl_z;
    TCP_STATUS status;

    data_ctrl_z = pvPortMalloc(strlen(data_string) + 2);//malloc(strlen(data_string) + 2);

    if(data_ctrl_z == NULL) {return TCP_FAIL_MEM;}

    // memset(data_ctrl_z , '\0',120);
    if (pdTRUE == SendATcommand("AT+CIPSEND", ">", 10000))
    {
        sprintf(data_ctrl_z, "%s%c", data_string, 26);
        if (pdFALSE == SendATcommand(data_ctrl_z, "SEND OK", 20000))
        {
            status = TCP_SEND_TIMEOUT;
        }
        else
        {
            status = TCP_SEND_SUCCESS;
        }
    }
    else
    {
        status = TCP_SEND_FAIL;
    }
    vPortFree(data_ctrl_z);

    return status;
}

TCP_STATUS TCP_GetStatus(void)
{

    if (pdTRUE == SendATcommand("AT+CIPSTATUS", "CONNECT OK", 2000))
    {
        return TCP_CONNECT_SUCCESS;
    }
    else
    {
        return TCP_CONNECT_FAIL;
    }
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
    if (pdTRUE == SendATcommand("AT+CIPCLOSE", "CLOSE OK", 10000))
    {
        return TCP_SUCCESS;
    }
    return TCP_FAIL;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : HTTP init
 * Description   :
 * command :
 * 
 *       AT+HTTPINIT
 *       AT+HTTPSSL=1
 *       AT+HTTPPARA="CID",1
 * This function
 *
 *END**************************************************************************/
HTTP_STATUS HTTP_Init(char *server)
{
    char command[256] ;
    uint8_t eRet = pdTRUE;
	
    SendATcommand("AT+HTTPTERM","OK",1000);
	
	  Config_GPRS_SIM908();
		//SendATcommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"","OK",2000);
    eRet = SendATcommand("AT+HTTPINIT","OK",1000);
    if(eRet == pdTRUE)
    {
        eRet = SendATcommand("AT+HTTPPARA=\"CID\",1","OK",1000);
        if(eRet != pdTRUE)
        {
            return HTTP_INIT_FAIL;
        }
        sprintf(command, "AT+HTTPPARA=\"URL\",\"%s\"", server);
        SendATcommand(command,"OK",1000);
        SendATcommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"","OK",2000);     

        return HTTP_INIT_SUCCESS;
    }

    return HTTP_INIT_FAIL;
   
}

/*FUNCTION**********************************************************************
 *
 * Function Name : HTTP Post
 * Description   :
 * command :
 * 
 *       AT+HTTPPARA="URL","http://ptsv2.com/t/Hao/post"
 *       AT+HTTPDATA=10,100000
 *       AT+HTTPPARA="CONTENT","application/json"
 *       AT+HTTPACTION=1
 * This function
 *
 *END**************************************************************************/
HTTP_STATUS HTTP_Post(char * data, uint32_t timeout)
{
    char command[50] = {0,};
    HTTP_STATUS httpStatus = HTTP_POST_SUCCESS;
    TickType_t xtime;
    
    xtime = xTaskGetTickCount();
    do
    {
    
    } while (xTaskGetTickCount() - xtime < 100 ) ;
    sprintf(command, "AT+HTTPDATA=%d,%d",strlen(data) + 1,timeout);
    if (pdTRUE == SendATcommand(command, "DOWNLOAD", 2000))
    {
        if (pdTRUE != SendATcommand(data, "OK", 20000))
        {
            httpStatus = HTTP_POST_FAIL;
        }
        else{
            SendATcommand("AT+HTTPACTION=1" , "+HTTPACTION" ,2000);
        }

    }
    else
    {
        httpStatus = HTTP_POST_FAIL ;
    }
    
    return httpStatus;
    
}


HTTP_STATUS HTTP_POST_FromSD(GPS_INFO gpsData, uint32_t sector_num, uint32_t data_size, uint32_t timeout, void (*func)(uint32_t , char *))
{
	char *command  = pvPortMalloc(30); 
    char *gpsBuff  = pvPortMalloc(160); 
    uint32_t i;
    HTTP_STATUS httpStatus = HTTP_POST_SUCCESS;

    TickType_t xtime;
    /* Need to delay before post */
    xtime = xTaskGetTickCount();
    do
    {
    
    } while (xTaskGetTickCount() - xtime < 50 ) ;

    sprintf(command, "AT+HTTPDATA=%d,%d",data_size + sector_num - 2 + 4 ,timeout);
    if (pdTRUE == SendATcommand(command, "DOWNLOAD", 2000))
    {
        for(i = 0; i < sector_num; i++)
        {
            func(i , gpsBuff);
			if((i < sector_num - 1) && (i > 0))
				printf("%s,",gpsBuff );
			else
				printf("%s",gpsBuff );
        }
		//tmpsize += sector_num -2 ;
        if (pdTRUE != SendATcommand("\n]}", "OK", 20000))
        {
            httpStatus = HTTP_POST_FAIL;
        }

        SendATcommand("AT+HTTPACTION=1", "OK", 2000) ;
    }
    else
    {
        httpStatus = HTTP_POST_FAIL;
    }    
		
	vPortFree(command);
	vPortFree(gpsBuff);
    return httpStatus;
}

		
		
/*FUNCTION**********************************************************************
 *
 * Function Name : HTTP Read
 * Description   : Read data response after post
 * command :
 *       AT+HTTPREAD
 * This function
 *
 *END**************************************************************************/
HTTP_STATUS HTTP_Read(char * datOut)
{
 //    char SIM_RxChar;
 //    char *buffer = pvPortMalloc(160);
	// volatile int cnt = 0;
 //    uint32_t error = pdTRUE;
    HTTP_STATUS httpStatus = HTTP_READ_FAIL;

    TickType_t xtime;
    /* Need to delay before post */
    xtime = xTaskGetTickCount();
    do
    {
    
    } while (xTaskGetTickCount() - xtime < 1000 ) ;

 //   memset(buffer, 0 , 160);
    if (pdTRUE == SendATcommand("AT+HTTPREAD", datOut, 10000))
    {
        #if 0
    	do {
			if (pdTRUE == xSerialGetChar(&uart2_handle, (signed char*)&SIM_RxChar, 5000))
			{
				*(buffer + cnt++) = SIM_RxChar;
			}
            else
            {
                break;
            }
            if (strstr(buffer , "OK\r\n"))
            {
                httpStatus = HTTP_READ_SUCCESS;
                break;
            }
    	}while(1) ;
        
		if(strstr(buffer , "<h2>404</h2>"))
		{
			httpStatus = HTTP_NOT_FOUND;
		}
		else if(strstr(buffer , "<h2>400</h2>"))
		{
			httpStatus = HTTP_PARAM_INVALID;
		}
		else if (strstr(buffer , "\r\nOK\r\n"))
		{
			httpStatus = HTTP_READ_SUCCESS;
		}

        if(httpStatus  == HTTP_READ_SUCCESS)
        {
            //strcpy(datOut , buffer);
        }
        else
        {
        	//strcpy(datOut , "ERROR");
        }
        #endif
        
        httpStatus = HTTP_READ_SUCCESS;

    }

    //strcpy(datOut , buffer);

    //vPortFree(buffer);
    return httpStatus;


}

/*FUNCTION**********************************************************************
 *
 * Function Name : HTTP Release
 * Description   :
 * command :
 * 
 *       AT+HTTPTERM
 * This function
 *
 *END**************************************************************************/
HTTP_STATUS HTTP_Release()
{
    if (pdTRUE == SendATcommand("AT+HTTPTERM","OK",1000))
    {
        return HTTP_RELEASE_SUCCESS;
    }

    return HTTP_RELEASE_FAIL;


}
uint8_t GetAccount()
{
    char buffer_acc[160] , *ptr_buff;
    uint16_t cnt = 0;
    char SIM_RxChar;

    ptr_buff = buffer_acc;

    if (SendATcommand("AT+CUSD=1,\"*101#\"", "OK", 2000))
    {
        do {
            if (pdFALSE != xSerialGetChar(&uart2_handle, (signed char*)&SIM_RxChar, 0xffff))
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
 //   Sim908_power_on(); // Power up Sim908 module
    //GetIMEI(imei);
    //GetAccount();
    /*****Config Sim908 Module *****************************/
    SendATcommand("AT+CFUN=1", "OK", 2000);       // off echo
    SendATcommand("AT+CIPSHUT", "SHUT OK", 3000); // disconect gprs
    SendATcommand("AT+CSCLK=1", "OK", 2000);      // sleep mode
    SendATcommand("AT+CMGF=1", "OK", 2000);
    // GPIO_WriteLow(DTR_GPIO_PORT, (GPIO_Pin_TypeDef)DTR_GPIO_PINS); //wake up
    // Power up GPS
    //SendATcommand("AT+CGPSPWR=1", "OK", 2000); // power up gps
    // Reset GPS Cold mde
    //SendATcommand("AT+CGPSRST=1", "OK", 2000);
    /************End Config Sim908 Module *****************************/
    // delay(1000);
    SendATcommand("AT&W", "OK", 2000);
}

void Sim908_power_on(void)
{

    if (pdFALSE == SendATcommand("AT", "OK", 2000))
    { // power on pulse
        SIM908_PWRON;
        delay_ms(3000);
        SIM908_PWROFF;
        // Wake up
        // waits for an answer from the module
        SendATcommand("ATE0", "OK", 2000);
        while(pdFALSE == SendATcommand("AT", "OK", 2000));
    }
    //SendATcommand("AT", "OK", 2000);
}

void GetCmdDataSIM(char *str , char DATA_AT[5][10])
{
    char * pch;
    int i=0;
    pch = strtok (str,":");

    while (pch != NULL)
    {
        pch = strtok(NULL,",\"\r\n");
        strcpy(DATA_AT[i++],pch);
        if(i == 5) break;
    }
}

/*Get Cell ID*/
void GetCellid(GPS_INFO  *info_cellid )
{
    char buff[32];
    char DATA_AT[5][10] ;
    signed char SIM_RxChar;
    
    if( xSemaphoreTake( xMutex, ( TickType_t ) portMAX_DELAY ) == pdTRUE )
    {
        memset(DATA_AT , '\0' , sizeof(DATA_AT));
        while(pdTRUE == xSerialGetChar(&uart2_handle, &SIM_RxChar,100));        
        printf("AT+CREG?\r");
        if(GetResponse(buff, 2000))
        {
            GetCmdDataSIM(buff ,DATA_AT);
            strcpy(info_cellid->LAC,strtok (DATA_AT[2],"\""));
            strcpy(info_cellid->CELLID ,strtok (DATA_AT[3],"\""));
            strcpy(info_cellid->latitude ,"0.00");
            strcpy(info_cellid->longtitude ,"0.00");
            strcpy(info_cellid->date ,"0");
        }
        xSemaphoreGive( xMutex );
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : GetIMEI
 * Description   : GetIMEI of Sim module
 * This function use to get id IMEI of Sim module
 *
 *END**************************************************************************/
uint8_t GetIMEI(char * imei)
{
    char buff[32];
    uint32_t error;
    signed char SIM_RxChar;
    
    if( xSemaphoreTake( xMutex, ( TickType_t ) portMAX_DELAY ) == pdTRUE )
    {
        while(pdTRUE == xSerialGetChar(&uart2_handle, &SIM_RxChar,100));
        printf("AT+GSN\r");
        if(GetResponse(buff, 2000))
        {
            strncpy(imei,strstr(buff,"\r\n") + 2,15);
            *(imei+15) = 0;
            error =  pdTRUE;
        }
        else
        {
            error =  pdFALSE;
        }
        xSemaphoreGive( xMutex );        
    }
    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : jsonDataPost
 * Description   : covert gps data to json string
 *
 *END**************************************************************************/
void jsonDataPost(GPS_INFO gpsData,char *outBuffer)
{

    char jsonString[160];

sprintf(jsonString , \
         "{\
\"id\":%s,\
\"lat\":%s,\
\"lng\":%s,\
\"speed\":%d,\
\"fuel\":%d,\
\"bearing\":%d\
}",gpsData.IMEI,gpsData.latitude,gpsData.longtitude,0,0,0);

    sprintf(outBuffer, "%s",jsonString);

}