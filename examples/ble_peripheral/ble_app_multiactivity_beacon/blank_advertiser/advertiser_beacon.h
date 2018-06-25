/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_adv_rfid Advertiser Module using the radio through the sd_radio_request SoftDevice API
 * @{
 * @ingroup ble_sdk_radio_time_slot_api
 * @brief Advertiser module. This module shows an example of using periodic timeslots on the radio when the SoftDevice is running
 *
 * @details This module implements an advertiser which can be run in parallel with the S110 (thus allowing, for example, to advertise while in a connection)
 *          This module shows an example of using periodic timeslots on the radio when the SoftDevice is running
 *
 * @note This module is experimental.
 *
 */
#ifndef ADVERTISER_rfid_H__
#define ADVERTISER_rfid_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble_types.h"
#include "ble_gap.h"
#include "ble_srv_common.h"
#include "nrf_soc.h"
//#include "app_error.h"

typedef enum {
    UESB_BITRATE_2MBPS = RADIO_MODE_MODE_Nrf_2Mbit, 
    UESB_BITRATE_1MBPS = RADIO_MODE_MODE_Nrf_1Mbit, 
    UESB_BITRATE_250KBPS = RADIO_MODE_MODE_Nrf_250Kbit
} uesb_bitrate_t;

typedef enum {
    UESB_CRC_16BIT = RADIO_CRCCNF_LEN_Two, 
    UESB_CRC_8BIT  = RADIO_CRCCNF_LEN_One, 
    UESB_CRC_OFF   = RADIO_CRCCNF_LEN_Disabled
} uesb_crc_t;

typedef struct
{
	uesb_bitrate_t          bitrate;       //空中速率
    uesb_crc_t              crc;           //CRC校验
    uint8_t                 rf_channel;    //射频频率
    uint8_t                 payload_length;//载荷长度
    uint8_t                 rf_addr_length;//    
    uint8_t                 rx_address_p0[5];
    uint8_t                 ployload_buf[32];//发送数据	
    uint32_t                adv_interval;
    ble_srv_error_handler_t error_handler;                      /**< Function to be called in case of an error. */
} ble_rfid_init_t;

#pragma pack(1) //让编译器对这个结构作1字节对齐
typedef struct{
	uint8_t rf_channel;     //<信道f=2400MHz + RF_CH×1.0MHz
	uint8_t datarate;       //<空中速率 0：1mbps ;2mbps; 250kbps。
	uint8_t crc_mode;       //<校验 0 ：CRC_OFF ；1：CRC_8BIT；2 ：CRC_16BIT。
	uint8_t adder_with;     //<地址长度 3：3 bytes； 4：4 bytes ; 5:5 bytes。
	uint8_t tx_address[5];  //<发送地址	
}RF_parameter;

typedef struct{
	uint16_t sleep_timer;   //<睡眠的时间 10~65536周期(使用RTC定时器)
}ST_parameter;

typedef struct{
	uint8_t ployload_len ;   //<发送数据长度 最大支持32
	uint8_t ployload_buf[32];//发送数据	
}PL_parameter;

typedef struct{
	uint8_t version_id;        //版本管理使用
	RF_parameter rf_parm;
	ST_parameter st_parm;
	PL_parameter pl_parm;
}radio_parm;
#pragma pack() //取消1字节对齐，恢复为默认4字节对齐

//软件版
#define     current_version_id  0X00 

/**@brief Function for handling system events.
 *
 * @details Handles all system events of interest to the rfid module. 
 *
 * @param[in]   event     received event.
 */
void app_rfid_on_sys_evt(uint32_t event);

/**@brief Function for initializing the advertiser module.
 *
 * @param[in]   p_init     structure containing advertiser configuration information.
 */
void app_rfid_init(ble_rfid_init_t * p_init);

/**@brief Function for starting the advertisement.
 *
 */
void app_rfid_start(void);

/**@brief Function for stopping the advertisement.
 * @note This functions returns immediately, but the advertisement is actually stopped after the next radio slot.
 *
 */
void app_rfid_stop(void);

#endif // ADVERTISER_RFID_H__
