#ifndef USER_DRV_CONSOLE_H
#define USER_DRV_CONSOLE_H

#include <stdint.h>
#include <string.h>
#include "debug_utils.h"
#include "user_config_sw_ver.h"

//软件版
#define     current_version_id  0X00 //< 
//commander_id的定义
#define 	RADIO_COMMANDER_MODE  0x00//<2.4g无线射频模式指令
#define     BLE_commander_mode	  0x01//<BLE 模式指令
#define     VERSION_INFO_INQUIRY  0x02//<版本信息查询
#define 	READ_BEACON_CMD					0x03//获取Beacon命名 
#define	  STOP_SCAN_BEACON				0x04//终止扫描beacon
#define	  STRAT_SCAN_BEACON				0x05//启动扫描Beacon


//报文域的掩码
#define HEAD1_MARK                  0x00 //
#define HEAD2_MARK                  0x01 //
#define PACK_LEN_MARK               0x02 //
#define COMMANDER_ID_MARK           0x03 //
#define DATA_BUF_MARK               0x04 //
#define CHECKSUM_MARK               0x05 //
#define BUSY_MASK                   0x06
/*-------------------错误码定义--------------------------------------------------*/
#define STATUS_SUCCESS      0x00//成功
#define PARM_INVALID        0x01//参数无效
#define CRC_FAILURE         0x02//校验失败
#define UNSUPPORTED         0x03//无法支持

typedef struct uart_protocol_interface_s uart_protocol_interface_t;

typedef void (*uart_event_received_data_handler_t)(const uint8_t * p_byte);
typedef uint8_t(*uart_event_check_data_hander_t)(uint8_t initcheckvalue,const uint8_t *p_value,uint8_t datalen);
typedef void (*uart_event_send_data_handler_t)(uint8_t cammander_tag,const uint8_t *devicedata,uint8_t datalen);
typedef void (*uart_event_analysis_data_handler)(void);

struct uart_protocol_interface_s
{  
    uart_event_check_data_hander_t       data_check_handler;    //数据校验函数	
	uart_event_received_data_handler_t   data_received_handler; //接收处理函数
	uart_event_send_data_handler_t       data_send_handler;     //发送处理函数
	uart_event_analysis_data_handler     data_analysis_handler; //数据解析函数
};

typedef struct uart_packer_info_s  uart_packer_info_t;

struct uart_packer_info_s
{
	uint8_t pack_RecState;   //接收状态
	uint8_t pack_cmd;        //报文_命令变量
	uint8_t pack_checksum;   //报文_校验和变量
	uint8_t pack_len_total;  //报文_数据域长度变量
	uint8_t pack_DatRev[256];//报文_数据域数据	
};

static uint8_t CalculateCheckValue(uint8_t initcheckvalue,const uint8_t *value_p,uint8_t datalen);
static void SendUartData(uint8_t cammander_tag,const uint8_t *devicedata,uint8_t datalen);
static void SendUartData(uint8_t cammander_tag,const uint8_t *devicedata,uint8_t datalen);
extern void uart_process(void);
extern void uart_receive_handle(const uint8_t * p_byte);
void rui_save_scan_beacon_sta(uint8_t sta);

#endif//USER_DRV_CONSOLE_H


