/* Standard includes. */
#include <stdio.h>

/* Library includes. */
#include "stm32f10x_lib.h"
#include "task.h"

typedef enum _tcp_status
{
    TCP_CONNECT_SUCCESS = 0 ,
    TCP_CONNECT_FAIL ,
    TCP_CONNECT_TIMEOUT,
    TCP_SEND_SUCCESS,
    TCP_SEND_FAIL ,
    TCP_SEND_TIMEOUT ,
    TCP_SUCCESS,
    TCP_FAIL,
    TCP_FAIL_MEM
} TCP_STATUS ;

typedef enum http_status
{
    HTTP_INIT_SUCCESS =0,
	HTTP_INIT_FAIL,
    HTTP_POST_SUCCESS,
    HTTP_POST_FAIL,
    HTTP_RELEASE_FAIL,
    HTTP_RELEASE_SUCCESS,
    HTTP_READ_SUCCESS,
    HTTP_READ_FAIL,
    HTTP_NOT_FOUND,
    HTTP_RESPONSE_FAILE,
    HTTP_PARAM_INVALID
} HTTP_STATUS ;

typedef struct gps_info_t
{
    char IMEI[20];
	char latitude[20];
	char longtitude[20];
	char date[20];
    char LAC[6];
    char CELLID[6];
    uint16_t  MNC;
    uint16_t  MCC;
    uint32_t  FIX;
    uint32_t  ONLINE;
}GPS_INFO;

#define MAX_LENGH_STR  128
#define SIM908_PWRON   GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET)
#define SIM908_PWROFF  GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET)
#define delay_ms(x)    vTaskDelay(x)

uint8_t GetResponse(char *buff_receive, uint32_t timeout);
int8_t SendATcommand(char *ATcommand, char *expected_answer, unsigned int timeout);
int8_t SendATcommand2(char *ATcommand,char *expected_answer,char *expected_answer2, unsigned int timeout);
uint8_t start_GPS(void);
BaseType_t Wait_GPS_Fix(void);
uint8_t get_GPS(GPS_INFO *vGPSinfo);
void Config_GPRS_SIM908(void);
TCP_STATUS TCP_Connect(char *IP_address, char *Port, unsigned int timeout);
TCP_STATUS TCP_Send(char *data_string)	;
TCP_STATUS TCP_Close(void);
TCP_STATUS TCP_GetStatus(void);

HTTP_STATUS HTTP_Init(char *server);
HTTP_STATUS HTTP_Post(char * data, uint32_t timeout);
HTTP_STATUS HTTP_Release();
HTTP_STATUS HTTP_Read(char * datOut);
HTTP_STATUS HTTP_POST_FromSD(GPS_INFO gpsData, uint32_t sector_num, uint32_t data_size, uint32_t timeout, void (*func)(uint32_t , char *));

void Sim908_setup(void);
void Sim908_power_on(void);
uint8_t GPS_PWR(void);
uint8_t GetAccount(void);
void GetCellid(GPS_INFO  *info_cellid );
void GetCmdDataSIM(char *str , char DATA_AT[5][10]);
uint8_t GetIMEI(char * imei);
void jsonDataPost(GPS_INFO gpsData,char *outBuffer);
//int8_t TCPSendATcommand(char *ATcommand, char *expected_answer,unsigned int timeout);

