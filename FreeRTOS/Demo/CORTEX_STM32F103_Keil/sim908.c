#if 1
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

/*Get response from SIMCOM after send AT command*/
uint8_t GetResponse(char *buff_receive, uint32_t timeout)
{
    uint8_t count_char = 0;
    signed char SIM_RxChar;
    char cPassMessage[MAX_LENGH_STR];
    do
    {
        if (pdFALSE != xSerialGetChar(&uart2_handle, &SIM_RxChar, timeout))
        {
            cPassMessage[count_char++] = SIM_RxChar;
        }
        else
        {
            return pdFALSE;
        }

    } while ((SIM_RxChar != '\r') && (count_char < MAX_LENGH_STR));

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
    printf("%s\r", ATcommand); // Send the AT command
    if (pdFALSE == GetResponse(buffer_response, timeout))
    {
        return pdFALSE;
    }

    if (strstr(buffer_response, expected_answer) == NULL)
    {
        return pdFALSE;
    }
    return pdTRUE;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : start_GPS
 * Description   : Starting GPS for modul SIM908
 * This function
 *
 *END**************************************************************************/
uint8_t start_GPS(void)
{
    // Wake up Module Sim
    // GPIO_WriteLow(PWKEY_GPIO_PORT, (GPIO_Pin_TypeDef)DTR_GPIO_PINS);
    // Power up GPS
    // sendATcommand("AT+CGPSPWR=1", "OK", 2000);
    // Reset GPS Hot mde
    // sendATcommand("AT+CGPSRST=1", "OK", 2000);
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
#if 0
uint8_t get_GPS(void)
{   
    char buffer_response[MAX_LENGH_STR];
    // First get the NMEA string
    printf("AT+CGPSINF=0\r");
    if(pdTRUE == GetResponse(buffer_response,"OK",2000))
    {
        strtok(buffer_response, ",");
        strcpy(longitude,strtok(NULL, ",")); // Gets longitude
        strcpy(latitude,strtok(NULL, ",")); // Gets latitude
        strcpy(altitude,strtok(NULL, ".")); // Gets altitude
        strtok(NULL, ",");
        strcpy(date,strtok(NULL, ".")); // Gets date
        strtok(NULL, ",");
        //free(buffer_gps_t) ;
        return pdTRUE ;
    }
    return pdFALSE;
}
#endif
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
 *
 *END**************************************************************************/
TCP_STATUS TCP_Connect(char *IP_address, char *Port)
{
    char command[70];
    memset(command, '\0', 70);
    sendATcommand("AT+CIPSHUT", "SHUT OK", 30000);
    // delay(2000) ;
    sprintf(command, "AT+CIPSTART=\"TCP\",\"%s\",\"%s\"", IP_address, Port);
    if (pdTRUE == sendATcommand(command, "CONNECT OK", 60000))
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
    if (1 == sendATcommand("AT+CIPSTATUS", "CONNECT OK", 20000))
    {
        // memset(data_ctrl_z , '\0',120);
        if (sendATcommand("AT+CIPSEND", ">", 20000))
        {
            sprintf(data_ctrl_z, "%s%c", data_string, 26);
            if (!sendATcommand(data_ctrl_z, "SEND OK", 30000))
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

void setup(void)
{
    power_on(); // Power up Sim908 module
                //    delay(10000); // wait
    // SentEnglis_SIMmsg("0944500186","123456");
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
    while (sendATcommand("AT+CREG?", "+CREG: 2,1", 2000) == 0)
        ; // Wait register to network
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

void power_on(void)
{
    uint8_t answer = 0;
    /* Initialize I/Os in Output Mode */
    // checks if the module is started

    answer = sendATcommand("AT", "OK", 2000);
    if (answer == 0)
    { // power on pulse
        SIM908_PWRON;
        delay_ms(3000);
        SIM908_PWROFF;
        // Wake up
        // GPIO_WriteLow(PWKEY_GPIO_PORT, (GPIO_Pin_TypeDef)DTR_GPIO_PINS);
        // waits for an answer from the module
        printf("ATE0\r");
        while (answer == 0)
        {
            // Send AT every two seconds and wait for the answer
            answer = sendATcommand("AT", "OK", 2000);
            // while(UART1_GetFlagStatus(UART1_FLAG_RXNE) == RESET);
        }
    }

    printf("ATE0\r");
}
#endif
