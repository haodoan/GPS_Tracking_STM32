/* Standard includes. */
#include <stdio.h>

/* Library includes. */
#include "stm32f10x_lib.h"
#include "task.h"

typedef enum _tcp_status
{
    TCP_SUCCESS = 0 ,
    TCP_CONNECT_FAIL ,
    TCP_SEND_FAIL ,
    TCP_SEND_TIMEOUT ,
    TCP_FAIL
} TCP_STATUS ;

#define MAX_LENGH_STR  100
#define SIM908_PWRON   GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET)
#define SIM908_PWROFF  GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET)
#define delay_ms(x)    vTaskDelay(x)

uint8_t GetResponse(char *buff_receive, uint32_t timeout);
int8_t sendATcommand(char *ATcommand, char *expected_answer, unsigned int timeout);
uint8_t start_GPS(void);
void Config_GPRS_SIM908(void);
TCP_STATUS TCP_Connect(char *IP_address, char *Port);
TCP_STATUS TCP_Send(char *data_string)	;
TCP_STATUS TCP_Close(void);
void Sim908_setup(void);
void Sim908_power_on(void);
