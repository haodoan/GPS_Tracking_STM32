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

typedef struct gps_info_t
{
	char latitude[20];
	char longtitude[20];
	char date[20];
    char LAC[5];
    char CELLID[5];
}GPS_INFO;
#define MAX_LENGH_STR  100
#define SIM908_PWRON   GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET)
#define SIM908_PWROFF  GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET)
#define delay_ms(x)    vTaskDelay(x)

uint8_t GetResponse(char *buff_receive, uint32_t timeout);
int8_t sendATcommand(char *ATcommand, char *expected_answer, unsigned int timeout);
uint8_t start_GPS(void);
BaseType_t Wait_GPS_Fix(void);
uint8_t get_GPS(GPS_INFO *vGPSinfo);
void Config_GPRS_SIM908(void);
TCP_STATUS TCP_Connect(char *IP_address, char *Port);
TCP_STATUS TCP_Send(char *data_string)	;
TCP_STATUS TCP_Close(void);
void Sim908_setup(void);
void Sim908_power_on(void);
uint8_t GPS_PWR(void);
uint8_t GetAccount(void);
void GetCellid(GPS_INFO  *info_cellid );
void GetCmdDataSIM(char *str , char DATA_AT[5][10]);
