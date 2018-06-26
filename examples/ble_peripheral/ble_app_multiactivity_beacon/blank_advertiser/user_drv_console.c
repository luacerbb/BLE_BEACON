#include "user_drv_console.h"
#include "app_uart.h"
#include "debug_utils.h"
#include "flash_opt.h"
#include "advertiser_beacon.h"
#include "app_scheduler.h"

#include "stdlib.h"

#include "scanner_beacon.h"

static uart_packer_info_t   m_uart_packer_info= {0};
static uint8_t m_lastmac[7] = {0};
uint8_t BeaconUnReadNum = 0; ///统计beacon数据未读次数，未读次数达到一定次数则判定为无需继续扫描beacon
extern uint8_t  m_beacon_start_sta[4];

uart_protocol_interface_t m_uart_protocol_interface=
{
	.data_check_handler=CalculateCheckValue,
	.data_received_handler=uart_receive_handle,
	.data_send_handler=SendUartData,
	.data_analysis_handler=uart_process,
};

/*******************************************************************************************************
 * 描  述 : 校验和处理函数
 * 入  参 : initcheckvalue 初始值  value_p需要校验的数据  datalen长度
 * 返回值 : 无
 *******************************************************************************************************/
static uint8_t CalculateCheckValue(uint8_t initcheckvalue,const uint8_t *value_p,uint8_t datalen)
{
	uint8_t i;
	for(i=0; i<datalen; i++)
	{
		initcheckvalue+=value_p[i];
	}
	return (uint8_t)initcheckvalue;
}

void nvm_write_radio_parm_info_write_complete_cb( uint32_t  result )
{
	uint8_t load_data[48];
	radio_parm temp_radio_parmeter;

	if (result == NRF_SUCCESS)
	{
		uint8_t i=0;
		DEBUG_PRINT(0,"pstorage UPDATE callback received \r\n");
		nvm_load(RADIO_PARM_BLOCK_ID,load_data,48,0);
		DEBUG_PRINT(0,"=======wait update load_data=======\r\n");
		for( i=0; i<sizeof(temp_radio_parmeter); i++)
		{
			DEBUG_PRINT(0,"%02x", load_data[i]);
		}

		memcpy(&temp_radio_parmeter.version_id,load_data,sizeof(temp_radio_parmeter));
		DEBUG_PRINT(0,"\r\n ============radio_parm=============\r\n");


		DEBUG_PRINT(0,"    version_id: %02x\r\n", temp_radio_parmeter.version_id);
		DEBUG_PRINT(0,"    rf_channel: %02d\r\n", temp_radio_parmeter.rf_parm.rf_channel);
		DEBUG_PRINT(0,"    datarate: %02x\r\n", temp_radio_parmeter.rf_parm.datarate);
		DEBUG_PRINT(0,"    adder_with: %02x\r\n", temp_radio_parmeter.rf_parm.adder_with);
		DEBUG_PRINT(0,"    tx_address[5]");
		for( i=0; i<5; i++)
		{
			DEBUG_PRINT(0," 0x%02x,", temp_radio_parmeter.rf_parm.tx_address[i]);
		}


		DEBUG_PRINT(0," \r\n    sleep_timer: %d\r\n", temp_radio_parmeter.st_parm.sleep_timer);


		DEBUG_PRINT(0,"    ployload_len: %d\r\n",temp_radio_parmeter.pl_parm.ployload_len);
		DEBUG_PRINT(0,"    ployload_buf[32]");
		for( i=0; i<32; i++)
		{
			DEBUG_PRINT(0,"%02x", temp_radio_parmeter.pl_parm.ployload_buf[i]);
		}
		DEBUG_PRINT(0,"\r\n ============radio_parm=============\r\n");

		NVIC_SystemReset();

	}
	else
	{
		DEBUG_PRINT(0,"pstorage UPDATE ERROR callback received \r\n");
	}
}

static void radio_parm_check()
{
	uint8_t statue;
#if defined(DEBUG) || defined(DEBUG_VERBOSE)
	uint8_t i;
#endif
	radio_parm *p_radio_parmeter;
	DEBUG_PRINT(0,"radio_parm_check()\r\n");
	if(m_uart_packer_info.pack_len_total!=sizeof(radio_parm))//传入蓝牙的广播ID长度是4
	{
		DEBUG_PRINT(0,"[ERROR] PARM_INVALID\r\n");
		statue=PARM_INVALID;
		SendUartData(m_uart_packer_info.pack_cmd,&statue,1);
		nrf_delay_ms(100);
		NVIC_SystemReset();			//重启复位,离开工厂模式和重新启用工厂模式时		
		return;
	}


	p_radio_parmeter=(radio_parm *)&m_uart_packer_info.pack_DatRev;

#if defined(DEBUG) || defined(DEBUG_VERBOSE)
	DEBUG_PRINT(0,"\r\n ============radio_parm [start]=============\r\n");

	DEBUG_PRINT(0,"    version_id: %02x\r\n", p_radio_parmeter->version_id);
	DEBUG_PRINT(0,"    rf_channel: %02d\r\n", p_radio_parmeter->rf_parm.rf_channel);
	DEBUG_PRINT(0,"    datarate: %02x\r\n", p_radio_parmeter->rf_parm.datarate);
	DEBUG_PRINT(0,"    adder_with: %02x\r\n", p_radio_parmeter->rf_parm.adder_with);
	DEBUG_PRINT(0,"    tx_address[5]");
	for(uint8_t i=0; i<5; i++)
	{
		DEBUG_PRINT(0," 0x%02x,", p_radio_parmeter->rf_parm.tx_address[i]);
	}
	DEBUG_PRINT(0," \r\n    sleep_timer: %d\r\n", p_radio_parmeter->st_parm.sleep_timer);


	DEBUG_PRINT(0,"    ployload_len: %d\r\n",p_radio_parmeter->pl_parm.ployload_len);
	DEBUG_PRINT(0,"    ployload_buf[32]");
	for( i=0; i<32; i++)
	{
		DEBUG_PRINT(0,"%02x", p_radio_parmeter->pl_parm.ployload_buf[i]);
	}
	DEBUG_PRINT(0,"\r\n ============radio_parm [end]=============\r\n");
#endif //defined(DEBUG) || defined(DEBUG_VERBOSE)  

	//支持版本检测
	if(p_radio_parmeter->version_id!=current_version_id)
	{
#if defined(DEBUG) || defined(DEBUG_VERBOSE)
		DEBUG_PRINT(0,"[ERROR] PARM_INVALID\r\n");
#endif //defined(DEBUG) || defined(DEBUG_VERBOSE)  
		statue=PARM_INVALID;
		goto radio_parm_check_end;
	}

	//支持的速率只有1Mbps、2Mbps、250Kbps三种
	if(p_radio_parmeter->rf_parm.datarate>UESB_BITRATE_250KBPS)
	{
#if defined(DEBUG) || defined(DEBUG_VERBOSE)
		DEBUG_PRINT(0,"[ERROR] PARM_INVALID\r\n");
#endif //defined(DEBUG) || defined(DEBUG_VERBOSE)  
		statue=PARM_INVALID;
		goto radio_parm_check_end;
	}

	//CRC支持的是OFF、CRC8、CRC16三种
	if(p_radio_parmeter->rf_parm.crc_mode>RADIO_CRCCNF_LEN_Two)
	{
#if defined(DEBUG) || defined(DEBUG_VERBOSE)
		DEBUG_PRINT(0,"[ERROR] PARM_INVALID\r\n");
#endif //defined(DEBUG) || defined(DEBUG_VERBOSE)  
		statue=PARM_INVALID;
		goto radio_parm_check_end;
	}

	//地址长度支持的是3、4、5的三种
	if((p_radio_parmeter->rf_parm.adder_with>5)||(p_radio_parmeter->rf_parm.adder_with<3))
	{
#if defined(DEBUG) || defined(DEBUG_VERBOSE)
		DEBUG_PRINT(0,"[ERROR] PARM_INVALID\r\n");
#endif //defined(DEBUG) || defined(DEBUG_VERBOSE)  
		statue=PARM_INVALID;
		goto radio_parm_check_end;
	}

	//ployload支持的最大长度为32
	if(p_radio_parmeter->pl_parm.ployload_len>32)
	{
#if defined(DEBUG) || defined(DEBUG_VERBOSE)
		DEBUG_PRINT(0,"[ERROR] PARM_INVALID\r\n");
#endif //defined(DEBUG) || defined(DEBUG_VERBOSE)  
		statue=PARM_INVALID;
		goto radio_parm_check_end;
	}

#if defined(DEBUG) || defined(DEBUG_VERBOSE)
	DEBUG_PRINT(0,"[ERROR] STATUS_SUCCESS\r\n");
#endif //defined(DEBUG) || defined(DEBUG_VERBOSE)  

	statue=STATUS_SUCCESS;
	nvm_write(RADIO_PARM_BLOCK_ID,m_uart_packer_info.pack_DatRev,48,0,nvm_write_radio_parm_info_write_complete_cb);
	SendUartData(m_uart_packer_info.pack_cmd,&statue,1);
	return;
radio_parm_check_end:
	SendUartData(m_uart_packer_info.pack_cmd,&statue,1);
	nrf_delay_ms(100);
	NVIC_SystemReset();			//重启复位,离开工厂模式和重新启用工厂模式时		
	
}

static void nvm_write_baby_id_info_write_complete_cb( uint32_t  result )
{
	uint8_t load_data[48];
	if (result == NRF_SUCCESS)
	{
		DEBUG_PRINT(0,"pstorage UPDATE callback received \r\n");

		nvm_load(ADV_DATA_BLOCK_ID,load_data,48,0); 
		DEBUG_PRINT(0,"    wait update load_data: %02x", load_data[0]);
		DEBUG_PRINT(0," %02x", load_data[1]);
		DEBUG_PRINT(0," %02x", load_data[2]);
		DEBUG_PRINT(0," %02x\n", load_data[3]);
		nrf_delay_ms(10);
		NVIC_SystemReset();			//重启复位,离开工厂模式和重新启用工厂模式时?
	}
	else
	{
		DEBUG_PRINT(0,"pstorage UPDATE ERROR callback received \r\n");
	}
}

static void nvm_write_beacon_scan_sta_complete_cb(uint32_t result)
{
	uint8_t load_data[4];
	if (result == NRF_SUCCESS)
	{
		DEBUG_PRINT(0,"pstorage beacon scan callback received \r\n");

		nvm_load(BEACON_SCAN_STA,load_data,4,0); 
		memcpy(m_beacon_start_sta,load_data,4);
		DEBUG_PRINT(0,"    wait update load_data: %02x", load_data[0]);
		DEBUG_PRINT(0," %02x", load_data[1]);
		DEBUG_PRINT(0," %02x", load_data[2]);
		DEBUG_PRINT(0," %02x\n", load_data[3]);
	//	nrf_delay_ms(100);
	//	NVIC_SystemReset();			//重启复位,离开工厂模式和重新启用工厂模式时?
	}
	else
	{
		DEBUG_PRINT(0,"pstorage UPDATE beacon scan ERROR \r\n");
	}
	
	
}

static void ble_parm_check()
{
	uint8_t statue;

	DEBUG_PRINT(0,"ble_parm_check()\r\n");
	if(m_uart_packer_info.pack_len_total!=4)//传入蓝牙的广播ID长度是4
	{
		DEBUG_PRINT(0,"[ERROR] PARM_INVALID\r\n");
		statue=PARM_INVALID;
		SendUartData(m_uart_packer_info.pack_cmd,&statue,1);
		nrf_delay_ms(100);
		NVIC_SystemReset();			//重启复位,离开工厂模式和重新启用工厂模式时		
	}
	else
	{
		DEBUG_PRINT(0,"[ERROR] STATUS_SUCCESS\r\n");
	  nvm_write(ADV_DATA_BLOCK_ID,m_uart_packer_info.pack_DatRev,48,0,nvm_write_baby_id_info_write_complete_cb);
		statue=STATUS_SUCCESS;
		SendUartData(m_uart_packer_info.pack_cmd,&statue,1);
	}

}

/*******************************************************************************************************
 * 描  述 : 串口输出数据
 * 入  参 : cammander_tag指令 devicedata数据域的数据 datalen数据长度
 * 返回值 : 无
 *******************************************************************************************************/
static void SendUartData(uint8_t cammander_tag,const uint8_t *devicedata,uint8_t datalen)
{
	uint8_t i=0;
	volatile uint8_t CheckValue=0;

	CheckValue=CalculateCheckValue(0,devicedata,datalen);
	CheckValue=CalculateCheckValue(CheckValue,&datalen,1);
	CheckValue=CalculateCheckValue(CheckValue,&cammander_tag,1);

	app_uart_put(0xAA);
	app_uart_put(0x55);
	app_uart_put(cammander_tag);
	app_uart_put(CheckValue);
	app_uart_put(datalen);
	for(i=0; i<datalen; i++)
	{
		app_uart_put(devicedata[i]);
	}

	m_uart_packer_info.pack_RecState = HEAD1_MARK;
	m_uart_packer_info.pack_len_total=0;

}

/**
 * @brief Handler timer event scheduling.
 */
static void uart_schedule_hander(void * p_event_data, uint16_t event_size)
{
	m_uart_protocol_interface.data_analysis_handler();
}

extern uint8_t m_last_maclist[MAC_LIST_MAX][7];
extern void beacon_next_scan_timer_start(uint16_t sec);
extern uint8_t m_last_beacon_num ; 
extern void beacon_scan_timer_stop(void);

static void rui_schedule_save_beacon_sta(void * p_event_data, uint16_t event_size)
{
		 rui_save_scan_beacon_sta(8);
}
void rui_save_scan_beacon_sta(uint8_t sta)
{
	__align(4) static uint8_t beacon_sta[4] = {0};
	//uint8_t beacon_sta[4] = {0};

	DEBUG_PRINT(0,"ready save beacon sta:%d\r\n",sta);
	memset(beacon_sta,sta,4);
	
	DEBUG_PRINT(0,"TEST val :%d,%d,%d,%d",beacon_sta[0],beacon_sta[1],beacon_sta[2],beacon_sta[3]);
	nvm_write(BEACON_SCAN_STA,beacon_sta,4,0,nvm_write_beacon_scan_sta_complete_cb); ///test

}
void uart_process()
{
	if(m_uart_packer_info.pack_RecState==BUSY_MASK)
	{
		volatile uint8_t CheckValue=0;
		uint8_t statue;
		uint8_t send_buf[30];
		uint8_t temp;
		uint8_t beacon_len;
	//	uint8_t beacon_sta[4] ={1};

		DEBUG_PRINT(0,"=====uart receive print start=====\r\n");
		DEBUG_PRINT(0,"pack_cmd :%02x \r\n",m_uart_packer_info.pack_cmd);
		DEBUG_PRINT(0,"pack_checksum :%02x \r\n",m_uart_packer_info.pack_checksum);
		DEBUG_PRINT(0,"pack_len_total :%02x \r\n",m_uart_packer_info.pack_len_total);
		DEBUG_PRINT(0,"pack_DatRev :");
		for(temp=0; temp<m_uart_packer_info.pack_len_total; temp++)
		{
			DEBUG_PRINT(0,"%02x",m_uart_packer_info.pack_DatRev[temp]);
		}
		DEBUG_PRINT(0,"\r\n=====nuart receive print end=====\r\n");


		CheckValue=CalculateCheckValue(0,m_uart_packer_info.pack_DatRev,m_uart_packer_info.pack_len_total);
		CheckValue=CalculateCheckValue(CheckValue,&m_uart_packer_info.pack_len_total,1);
		CheckValue=CalculateCheckValue(CheckValue,&m_uart_packer_info.pack_cmd,1);

		if( CheckValue!=m_uart_packer_info.pack_checksum)
		{
			DEBUG_PRINT(0,"[ERROR] CRC_FAILURE\r\n");
			statue=CRC_FAILURE;
			SendUartData(m_uart_packer_info.pack_cmd,&statue,1);
			nrf_delay_ms(100);
			NVIC_SystemReset();			//重启复位,离开工厂模式和重新启用工厂模式时		
		}
		else
		{
			switch(m_uart_packer_info.pack_cmd)
			{
			case RADIO_COMMANDER_MODE:
				radio_parm_check();

				break;
			case BLE_commander_mode:
				ble_parm_check();

				break;
			case VERSION_INFO_INQUIRY:
				send_buf[0]=STATUS_SUCCESS;
				memcpy(&send_buf[1],WATCH_SW_VERSION,sizeof(WATCH_SW_VERSION)-1);
				SendUartData(m_uart_packer_info.pack_cmd,send_buf,sizeof(WATCH_SW_VERSION));	
				//nvm_write(BEACON_SCAN_STA,beacon_sta,4,0,nvm_write_beacon_scan_sta_complete_cb);
				//nrf_delay_ms(100);
				//NVIC_SystemReset();			//重启复位,离开工厂模式和重新启用工厂模式时						
				break;
			case READ_BEACON_CMD:
				///清除未读beacon次数
				BeaconUnReadNum = 0;
				send_buf[0]=STATUS_SUCCESS;
				temp = abs(m_lastmac[6] - m_last_maclist[0][6]);
			
				if(memcmp(m_lastmac,&m_last_maclist[0][0],6) != 0 || temp > 10)
				{
						beacon_len = m_last_beacon_num * 7;
						memcpy(&send_buf[1],m_last_maclist[0],beacon_len);
					  memcpy(m_lastmac,&m_last_maclist[0][0],7);
				}
				else
				{
						beacon_len  = 0;
				}	
				
				DEBUG_PRINT(0,"beacon data len:%d,%d,%d,%d\r\n",beacon_len,m_last_beacon_num,m_lastmac[6],m_last_maclist[0][6]);
				DEBUG_PRINT(0,"beacon data val=%d,%02x:%02x:%02x:%02x:%02x:%02x\r\n",temp,m_lastmac[0],m_lastmac[1],m_lastmac[2],m_lastmac[3],m_lastmac[4],m_lastmac[5]);
				SendUartData(m_uart_packer_info.pack_cmd,send_buf,beacon_len + 1);				
				//nrf_delay_ms(100);
				//NVIC_SystemReset();			//重启复位,离开工厂模式和重新启用工厂模式时						
				
				///避免异常情况下 执行读取BEACON，但扫描时间为零，定时器未开启
				if(!m_beacon_start_sta[0])
				{
				   beacon_next_scan_timer_start(60);//默认60秒
				}
				
				break;
			case STOP_SCAN_BEACON:
					if(!m_uart_packer_info.pack_DatRev[0])
					{
						 ///停止BEACON扫描
						send_buf[0]=STATUS_SUCCESS;
						rui_save_scan_beacon_sta(0);

				  	SendUartData(m_uart_packer_info.pack_cmd,send_buf,1);
						beacon_scan_timer_stop();
					}	
				break;
			case STRAT_SCAN_BEACON:
				{
					 ///启动beacon 扫描定时器
					 send_buf[0]=STATUS_SUCCESS;
					 memset(m_lastmac,0,sizeof(m_lastmac));
					 rui_save_scan_beacon_sta((uint8_t)m_uart_packer_info.pack_DatRev[0]);

					 SendUartData(m_uart_packer_info.pack_cmd,send_buf,1);
					 beacon_next_scan_timer_start(m_uart_packer_info.pack_DatRev[0]);
				}
				break;
			default:
				statue=UNSUPPORTED;
				SendUartData(m_uart_packer_info.pack_cmd,&statue,1);
				nrf_delay_ms(100);
				NVIC_SystemReset();			//重启复位,离开工厂模式和重新启用工厂模式时		
				break;
			}
		}

	}
}

void uart_receive_handle(const uint8_t * p_byte)
{
	static uint8_t len=0;                //已接收的数据长度
	uint32_t       err_code;
	//ASSERT(p_byte);

	switch(m_uart_packer_info.pack_RecState)
	{
	case HEAD1_MARK:
		if( *p_byte == 0xAA )
		{
			m_uart_packer_info.pack_RecState = HEAD2_MARK;
		}
		break;

	case HEAD2_MARK:
		if( *p_byte == 0x55)
		{
			m_uart_packer_info.pack_RecState = COMMANDER_ID_MARK;
		}
		else if(*p_byte == 0xAA)
		{
			m_uart_packer_info.pack_RecState = HEAD2_MARK;
		}
		break;

	case COMMANDER_ID_MARK:
		m_uart_packer_info.pack_cmd = *p_byte;
		m_uart_packer_info.pack_RecState = CHECKSUM_MARK;
		break;

	case CHECKSUM_MARK:
		m_uart_packer_info.pack_checksum = *p_byte;
		m_uart_packer_info.pack_RecState = PACK_LEN_MARK;
		break;

	case PACK_LEN_MARK:
		m_uart_packer_info.pack_RecState = DATA_BUF_MARK;
		m_uart_packer_info.pack_len_total =*p_byte;
		len=0;
		if(m_uart_packer_info.pack_len_total== 0)
		{
			m_uart_packer_info.pack_RecState = BUSY_MASK;
			
			err_code = app_sched_event_put(0, 0, uart_schedule_hander);
			APP_ERROR_CHECK(err_code);
		}
		break;

	case DATA_BUF_MARK:
		m_uart_packer_info.pack_DatRev[len]=*p_byte;
		len++;
		if(len==m_uart_packer_info.pack_len_total)
		{
			m_uart_packer_info.pack_RecState = BUSY_MASK;
			err_code = app_sched_event_put(0, 0, uart_schedule_hander);
			APP_ERROR_CHECK(err_code);
		}
		break;

	case BUSY_MASK:
		break;

	default:
		break;
	}
}

