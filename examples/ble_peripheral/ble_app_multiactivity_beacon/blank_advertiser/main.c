/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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
/**
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_uart.h"

#include "debug_utils.h"

#include "flash_opt.h"

#include "app_timer_appsh.h"
#include "app_scheduler.h"

#include "nrf_gpio.h"
#include "app_gpiote.h"

#include "app_button.h"

///test
#include "scanner_beacon.h"


#if USE_WDT_MODULE
#include "nrf_drv_wdt.h"
#endif   //USE_WDT_MODULE

/*Addition to do radio rfid at all time*/
#include "advertiser_beacon.h"

///hou test
#include "advertiser_ibeacon.h"


#include "user_drv_console.h"

#include "user_config_sw_ver.h"

extern uart_protocol_interface_t m_uart_protocol_interface;

#define IS_SRVC_CHANGED_CHARACT_PRESENT      0                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define APP_COMPANY_IDENTIFIER               0x0059                                     /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */

#define RADIO_RFID_INTERVAL                  1000                                        /**< The radio RFIC's advertising interval, in milliseconds*/

/*end addition for rfid*/


#define APP_ADV_INTERVAL                     MSEC_TO_UNITS(1000, UNIT_0_625_MS)          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 1000 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS           BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED       /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE              4                                          /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(500, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(1000, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                        0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY        APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT         3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                       1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                       0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                            0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static uint16_t                              m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */


static volatile bool update_request = false;

#define APP_BABY_ID_INFO_LENGTH          0x04                              /**< Total length of information advertised by the baby_id. */
__align(4) uint8_t m_baby_id_info[APP_BABY_ID_INFO_LENGTH] =  {0x11,0x22,0x33,0x44};

#define APP_RADIO_PARM_INFO_LENGTH       48 //sizeof(radio_parm)即45+3
__align(4) uint8_t m_radio_parm_info[APP_RADIO_PARM_INFO_LENGTH];

#define UART_TX_BUF_SIZE                256                                        /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                1                                         /**< UART RX buffer size. */

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE,\
                                            BLE_STACK_HANDLER_SCHED_EVT_SIZE)       /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

#define APP_GPIOTE_MAX_USERS            1  // Maximum number of users of the GPIOTE handler. 

#define BUTTON_DEBOUNCE_DELAY			50 // Delay from a GPIOTE event until a button is reported as pushed. 


#define TX_POWER_LEVEL                 (4)/**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */
#ifdef USE_WDT_MODULE
nrf_drv_wdt_channel_id    m_channel_id;
#define WDT_INTERVAL      APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)            /**< wdt interval (ticks). */
APP_TIMER_DEF(m_wdt_timer_id);                                                  /**< wdt timer. */
#endif   //USE_WDT_MODULE

#define BEACON_NEXT_SCAN_INTERVAL  APP_TIMER_TICKS(21000,APP_TIMER_PRESCALER)
APP_TIMER_DEF(m_start_scan_beacon_timer_id);

///test scan
static struct
{
    ble_uuid128_t                 uuid;
    bool                          keep_running;
    bool                          is_running;
    nrf_radio_request_t           timeslot_request;
    uint8_t                       scn_pdu[40];
    ble_scan_beacon_evt_handler_t evt_handler;
    ble_srv_error_handler_t       error_handler;                      /**< Function to be called in case of an error. */
} m_beacon_scanner;


__align(4)
static struct 
{
	uint8_t manu_uuid[16];
	uint16_t major;
	uint16_t minjor;
}m_beacon_save_data;    	

__align(4) uint8_t  m_beacon_start_sta[4] = {0}; ///为0 默认不启动beacon 扫描

#define BEACON_UNREAD_MAX 5
//#define MAC_LIST_MAX  20
uint8_t m_maclist[MAC_LIST_MAX][7] = {0}; //当前mac列表
uint8_t m_beacon_num = 0; ///扫描到beacon数量

uint8_t m_last_maclist[MAC_LIST_MAX][7] = {0}; ///上次
uint8_t m_last_beacon_num = 0; ///上次扫描到beacon数量


static volatile bool m_ready_send_mac = false;
extern uint8_t BeaconUnReadNum ; ///统计beacon数据未读次数，未读次数达到一定次数则判定为无需继续扫描beacon


void ble_wakeup_mtk_uart(void);
static void scanner_start();

void beacon_next_scan_timer_start(uint16_t sec);

/*
void app_beacon_scanner_init(ble_beacon_scanner_init_t * p_init)
{
    m_beacon_scanner.evt_handler = p_init->evt_handler;
    m_beacon_scanner.error_handler= p_init->error_handler;
}

/*
void app_beacon_scanner_start(void)
{
    uint32_t err_code;

    m_beacon_scanner.keep_running = true;
    m_beacon_scanner.is_running   = true;

    err_code = sd_radio_session_open(m_timeslot_callback);
    if ((err_code != NRF_SUCCESS) && (m_beacon_scanner.error_handler != NULL))
    {
        m_beacon_scanner.error_handler(err_code);
    }
    
    err_code = sd_radio_request(m_request_earliest(NRF_RADIO_PRIORITY_NORMAL));
    if ((err_code != NRF_SUCCESS) && (m_beacon_scanner.error_handler != NULL))
    {
        m_beacon_scanner.error_handler(err_code);
    }
}
*/
/*
void app_beacon_scanner_stop(void)
{
    m_beacon_scanner.keep_running = false;
}
*/



/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}



/**@brief check connection state..
*/
bool is_connected(void)
{
	return (m_conn_handle != BLE_CONN_HANDLE_INVALID);
}

void beacon_scan_timer_stop(void)
{
	uint32_t err_code;
	//return;
	BeaconUnReadNum = 0;
	err_code = app_timer_stop(m_start_scan_beacon_timer_id);
	APP_ERROR_CHECK(err_code);
	
	DEBUG_PRINT(0,"stop scan beacon timer\r\n");

}



#ifdef USE_WDT_MODULE
 
/**@brief Function for handling the WDT timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void wdt_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    nrf_drv_wdt_channel_feed(m_channel_id);
}
#endif //USE_WDT_MODULE

static void beacon_scan_start(void * p_context)
{
		m_beacon_num = 0;
	  memset(m_maclist,0,sizeof(m_maclist));
		
	  DEBUG_PRINT(0,"unReadCnt:%d\r\n",BeaconUnReadNum);
	
		if(BeaconUnReadNum++ < BEACON_UNREAD_MAX)
		{
			scanner_start();
		}
		else
		{
			beacon_scan_timer_stop();
			rui_save_scan_beacon_sta(0);
		}	
}


void rui_test_restart(void * p_context)
{
	nrf_delay_ms(100);
	NVIC_SystemReset();	
}
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
#ifdef USE_WDT_MODULE
	 // Create timers.
    err_code = app_timer_create(&m_wdt_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                wdt_timeout_handler);
	APP_ERROR_CHECK(err_code);
	
	    err_code = app_timer_create(&m_start_scan_beacon_timer_id,
                                APP_TIMER_MODE_REPEATED,//APP_TIMER_MODE_SINGLE_SHOT,
                                beacon_scan_start);
	APP_ERROR_CHECK(err_code);

	
//		    err_code = app_timer_create(&m_start_scan_beacon_timer_id,
//                                APP_TIMER_MODE_SINGLE_SHOT,//APP_TIMER_MODE_SINGLE_SHOT,
//                               rui_test_restart );

#endif //USE_WDT_MODULE
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);


    /* YOUR_JOB: Use an appearance value matching the application's use case.
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
    APP_ERROR_CHECK(err_code); */
	
///houqztest																					
		err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_TAG);
    APP_ERROR_CHECK(err_code);																			
																					
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
	
	err_code = sd_ble_gap_tx_power_set(TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Device Information services.
 */
static void services_init(void)
{
	uint32_t       err_code;
	ble_dis_init_t dis_init;

	// Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
	ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char*)MODEL_NUMBER);
	ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char*)HARDWARE_REVISION);
	ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char*)FIRMWARE_REVISION);
	ble_srv_ascii_to_utf8(&dis_init.sw_rev_str, (char*)SOFTWARE_REVISION);
	

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);
		
		///将设备相关信息写入BLE STACK DATABASE
    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
	#ifdef USE_WDT_MODULE
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_wdt_timer_id, WDT_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	#endif //USE_WDT_MODULE
	
	if(m_beacon_start_sta[0])
	{	
		//beacon_next_scan_timer_start();
		beacon_next_scan_timer_start(m_beacon_start_sta[0]);
	}
		///test
		//err_code = app_timer_start(m_start_scan_beacon_timer_id, WDT_INTERVAL, NULL);

}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for initializing the GPIOTE handler module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
	uint32_t       err_code;
	uint8_t data_read;

	switch (p_event->evt_type)
	{
	case APP_UART_DATA_READY:
		app_uart_get(&data_read);
		//DEBUG_PRINT(0,"    storage source_data_update\r\n");
		m_uart_protocol_interface.data_received_handler(&data_read);
		break;

	case APP_UART_COMMUNICATION_ERROR:
		APP_ERROR_HANDLER(p_event->data.error_communication);
		break;

	case APP_UART_FIFO_ERROR:
		APP_ERROR_HANDLER(p_event->data.error_code);
		break;

	default:
		break;
	}
}


/**@snippet [Handling the data received over UART] */
/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
	uint32_t                     err_code;
	const app_uart_comm_params_t comm_params =
	{
		RX_PIN_NUMBER,
		TX_PIN_NUMBER,
		RTS_PIN_NUMBER,
		CTS_PIN_NUMBER,
		APP_UART_FLOW_CONTROL_DISABLED,
		false,
		UART_BAUDRATE_BAUDRATE_Baud9600
	};

	APP_UART_FIFO_INIT( &comm_params,
	                    UART_RX_BUF_SIZE,
	                    UART_TX_BUF_SIZE,
	                    uart_event_handle,
	                    APP_IRQ_PRIORITY_LOW,
	                    err_code);
	APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */



/**@brief Function for disabling the UART when entering the UART_OFF state.
 */
static void action_uart_disable(void)
{
	
    NRF_UART0->TASKS_STOPTX = 1;
    NRF_UART0->TASKS_STOPRX = 1;
    NRF_UART0->ENABLE       = (UART_ENABLE_ENABLE_Disabled << UART_ENABLE_ENABLE_Pos);
	
	nrf_gpio_cfg_default(TX_PIN_NUMBER);
	nrf_gpio_cfg_default(RX_PIN_NUMBER);
}


/**@brief Function for enabling the UART when entering the UART_ON state.
 */ 
static void action_uart_enable(void)
{

    nrf_gpio_cfg_output(TX_PIN_NUMBER);
    nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);
	
	NRF_UART0->ENABLE        = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
	NRF_UART0->TASKS_STARTRX = 1;
	NRF_UART0->TASKS_STARTTX = 1;
}


/*
 * Handler to be called when button is pushed and release.
 * param[in]   pin_no 			The pin number where the event is genereated
 * param[in]   button_action 	Is the button pushed or released
 */
static void button_handler(uint8_t pin_no, uint8_t button_action)
{
		DEBUG_PRINT(0,"PIN=%d,ac=%d\r\n",pin_no,button_action);
    if(button_action == APP_BUTTON_PUSH)
    {
        switch(pin_no)
        {
            case PIN_BUTTON_UART:
			    action_uart_disable();
                break;
            default:
                break;
        }
    }
	if(button_action == APP_BUTTON_RELEASE)
	{
		 switch(pin_no)
        {
            case PIN_BUTTON_UART:
							action_uart_enable();
							if(m_ready_send_mac)
							{
									//m_uart_protocol_interface.data_send_handler(0xbb,m_maclist[0],7);
									m_ready_send_mac = false;
							} 	
                break;
            default:
                break;
        }
	}
}

static void action_uart_button_init(void)
{
	// Button configuration structure.
    static app_button_cfg_t p_button[] = {  {PIN_BUTTON_UART, BUTTON_UART_ACTIVE_STATE, NRF_GPIO_PIN_NOPULL, button_handler}};
	uint32_t      err_code;
	
		// Initializing the buttons.
	err_code = app_button_init(p_button, sizeof(p_button) / sizeof(p_button[0]), BUTTON_DEBOUNCE_DELAY);
	APP_ERROR_CHECK(err_code);
                                            
    // Enabling the buttons.										
	err_code = app_button_enable();
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
	uint32_t      err_code;
	ble_advdata_t advdata;
	
	ble_advdata_manuf_data_t manuf_specific_data;
	uint8_t manuf_data[26] = {0};
	
	///test
	uint8_t  manlen = 0;
	ble_advdata_t scanrspdata;
	memset(&scanrspdata,0,sizeof(ble_advdata_t));
	scanrspdata.name_type = BLE_ADVDATA_FULL_NAME;
	
	scanrspdata.include_appearance      = true;
	//scanrspdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

	///test
	manlen = app_beacon_get_manuf_data(manuf_data);
	 DEBUG_PRINT(0,"mamu len %d\r\n",manlen); 

	manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;  // 0X004C
	manuf_specific_data.data.p_data = (uint8_t *) manuf_data;
	manuf_specific_data.data.size   = manlen; ///FLAG + COMPANYID + BEACONID + DATALEN +UUID + MAJ+MIN(2BYTE) +RSSI
	
	memset(&advdata, 0, sizeof(advdata));

	 advdata.name_type               = BLE_ADVDATA_NO_NAME;
    advdata.include_appearance      = false;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.p_manuf_specific_data = &manuf_specific_data;

	err_code = ble_advdata_set(&advdata,&scanrspdata);//&scanrspdata
	

/*
	manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;  // 0X004C

	
	manuf_specific_data.data.p_data = (uint8_t *) m_baby_id_info;
	manuf_specific_data.data.size   = APP_BABY_ID_INFO_LENGTH;

	
	
	
	
	memset(&advdata, 0, sizeof(advdata));

	advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.p_manuf_specific_data = &manuf_specific_data;

	err_code = ble_advdata_set(&advdata,NULL);
	APP_ERROR_CHECK(err_code);
*/
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
	 ble_gap_addr_t gap_addr = {0};;
	
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
 
    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

	 adv_params.type        = BLE_GAP_ADV_TYPE_ADV_SCAN_IND;   ///ibeacon test 不让他连接,可获取到扫描应答数据
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
	
	
		///houtest
	  sd_ble_gap_address_get(&gap_addr);
	  
	  
		DEBUG_PRINT(0,"adv addr type:%d -- %02x:%02x:%02x:%02x:%02x:%02x\r\n",gap_addr.addr_type ,gap_addr.addr[0],gap_addr.addr[1],gap_addr.addr[2],gap_addr.addr[3],gap_addr.addr[4],gap_addr.addr[5]);	
	 
	 
}


/**@brief Function for stoping advertising.
 */
static void advertising_stop(void)
{
	uint32_t err_code;
	err_code = sd_ble_gap_adv_stop();	
	APP_ERROR_CHECK(err_code);
}
///houqz test
typedef struct
{
    uint8_t     * p_data;                                             /**< Pointer to data. */
    uint16_t      data_len;                                           /**< Length of data. */
}data_t;
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index+1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index+2];
            p_typedata->data_len = field_length-1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}

#define UUID16_EXTRACT(DST, SRC) \
    do                           \
    {                            \
        (*(DST))   = (SRC)[1];   \
        (*(DST)) <<= 8;          \
        (*(DST))  |= (SRC)[0];   \
    } while (0)




/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
	/*
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;            
            break;

        case BLE_GAP_EVT_DISCONNECTED:
			m_conn_handle = BLE_CONN_HANDLE_INVALID; // Set connection handle to 0xFFFF  
			advertising_start();
			app_beacon_start();
            break;
			
		case BLE_GAP_EVT_TIMEOUT:
			if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            {				
				advertising_start();
            }
			break;
			
        default:
            // No implementation needed.
            break;
    }
*/		
	
		uint32_t                err_code;
    const ble_gap_evt_t   * p_gap_evt = &p_ble_evt->evt.gap_evt;

	//	DEBUG_PRINT(0,"scan get %d\r\n",p_ble_evt->header.evt_id);

    switch (p_ble_evt->header.evt_id)
    {
				case BLE_GAP_EVT_CONNECTED:
					m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;     
					//app_beacon_start();				
					break;

        case BLE_GAP_EVT_ADV_REPORT:
        {
            data_t adv_data;
            data_t type_data;
						uint8_t i = 0;
						uint8_t addr[BLE_GAP_ADDR_LEN] = {0};       /**< 48-bit address, HSB format. */

					  //memcpy(addr,p_gap_evt->params.adv_report.peer_addr.addr,sizeof(addr));
						for(i = 0; i < BLE_GAP_ADDR_LEN; i ++)
						{
							  addr[i] = p_gap_evt->params.adv_report.peer_addr.addr[BLE_GAP_ADDR_LEN - i - 1];
						}
						//DEBUG_PRINT(0,"REPORT ADR TYPE :%d\r\n",p_gap_evt->params.adv_report.peer_addr.addr_type);
						//if(!p_gap_evt->params.adv_report.scan_rsp)
						//DEBUG_PRINT(0,"scan get report%02x:%02x:%02x:%02x:%02x:%02x rssi=%d\r\n",addr[0],addr[1],addr[2],addr[3],addr[4],addr[5],p_gap_evt->params.adv_report.rssi);
						
					  ///DATA PARSE
//						for(i = 0; i < p_gap_evt->params.adv_report.dlen; i++)
//						{
//							DEBUG_PRINT(0,"%02x ",p_gap_evt->params.adv_report.data[i]);
//						}
//						DEBUG_PRINT(0,"\r\n ");
				
//						DEBUG_PRINT(0,"type = %d\r\n ",p_gap_evt->params.adv_report.type);
	
						
            // Initialize advertisement report for parsing.
            adv_data.p_data = (uint8_t *)p_gap_evt->params.adv_report.data;
            adv_data.data_len = p_gap_evt->params.adv_report.dlen;
						
					//	DEBUG_PRINT(0,"scan rsp%d,%d\r\n",p_gap_evt->params.adv_report.scan_rsp,adv_data.data_len);
//						if(p_gap_evt->params.adv_report.scan_rsp && adv_data.data_len)
//						{
//								for(i = 0; i < p_gap_evt->params.adv_report.dlen; i++)
//								{
//									DEBUG_PRINT(0,"%02x ",p_gap_evt->params.adv_report.data[i]);
//								}
//								DEBUG_PRINT(0,"\r\n ");
//								
//								break;
//						
//						}
						///匹配厂商数据 houtest
						err_code = adv_report_parse(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
																			&adv_data,
																			&type_data);
																			
//						for(i = 0; i < type_data.data_len; i++)
//						{
//							DEBUG_PRINT(0,"%02x ",type_data.p_data[i]);
//						}
//						DEBUG_PRINT(0,"error %d \r\n ",err_code);

						if(err_code == NRF_SUCCESS)
						{
								if(type_data.p_data[2] == 0x02 && type_data.data_len > 3)
								{
										DEBUG_PRINT(0,"scan get report%02x:%02x:%02x:%02x:%02x:%02x rssi=%d\r\n",addr[5],addr[4],addr[3],addr[2],addr[1],addr[0],p_gap_evt->params.adv_report.rssi);
										
										  uint8_t i = 0;
											for(i = 0; i < m_beacon_num; i ++)
											{
													if(!memcmp(addr,m_maclist[i],6))
													{
															break;
													}	
											}
											
											if(i == m_beacon_num && i < MAC_LIST_MAX)
											{
												 uint8_t temp_rssi = abs(p_gap_evt->params.adv_report.rssi);
												 //memcpy(m_maclist[i],addr,sizeof(addr));
												 //m_maclist[i][6] = p_gap_evt->params.adv_report.rssi;
												for(i = 0; i <= m_beacon_num; i ++) 
												{
													if(temp_rssi < m_maclist[i][6] || !m_maclist[i][6])
													{
													   memcpy(m_maclist[i + 1],m_maclist[i],(m_beacon_num - i) * 7);
														 memcpy(m_maclist[i],addr,sizeof(addr));
												     m_maclist[i][6] = temp_rssi;
														 break;
													}	
												}
												 m_beacon_num ++ ;
												
												DEBUG_PRINT(0,"rssi test:%d\r\n",temp_rssi);
											}	
										  else
											{
												 DEBUG_PRINT(0,"invalid beacon\r\n");
													break;
											}	
											
											for(i = 0; i < m_beacon_num; i ++) 
											{
												DEBUG_PRINT(0,"RSSI:%d",m_maclist[i][6]);
											}
											DEBUG_PRINT(0,"\r\n");
										if(type_data.p_data[0] == 0x59 && type_data.p_data[1] == 0)
										{
												DEBUG_PRINT(0,"nordic beacon\r\n");
										}	
										else if (type_data.p_data[0] == 0x4c && type_data.p_data[1] == 0)
										{
												DEBUG_PRINT(0,"apple beacon\r\n");
										}	
										else
										{
												DEBUG_PRINT(0,"other beacon\r\n");										
										}
										
										for(i = 0; i < p_gap_evt->params.adv_report.dlen; i++)
										{
											DEBUG_PRINT(0,"%02x ",p_gap_evt->params.adv_report.data[i]);
										}
										DEBUG_PRINT(0,"\r\n ");
										
										ble_wakeup_mtk_uart();
								}
								
								break;
						}	
						
//            err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
//                                        &adv_data,
//                                        &type_data);

//            if (err_code != NRF_SUCCESS)
//            {
//                // Compare short local name in case complete name does not match.
//                err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
//                                            &adv_data,
//                                            &type_data);
//            }

            // Verify if short or complete name matches target.
            if (err_code == NRF_SUCCESS)
            {
                uint16_t extracted_uuid;

                // UUIDs found, look for matching UUID
                for (uint32_t u_index = 0; u_index < (type_data.data_len/2); u_index++)
                {
                    UUID16_EXTRACT(&extracted_uuid,&type_data.p_data[u_index * 2]);

                    DEBUG_PRINT(0,"\t[APPL]: %x\r\n",extracted_uuid);

                  /* if(extracted_uuid == TARGET_UUID)
                    {
                        // Stop scanning.
                        err_code = sd_ble_gap_scan_stop();

                        if (err_code != NRF_SUCCESS)
                        {
                            APPL_LOG("[APPL]: Scan stop failed, reason %d\r\n", err_code);
                        }
                        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
                        APP_ERROR_CHECK(err_code);

                        m_scan_param.selective = 0; 
                        m_scan_param.p_whitelist = NULL;

                        // Initiate connection.
                        err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
                                                      &m_scan_param,
                                                      &m_connection_param);

                        m_whitelist_temporarily_disabled = false;

                        if (err_code != NRF_SUCCESS)
                        {
                            APPL_LOG("[APPL]: Connection Request Failed, reason %d\r\n", err_code);
                        }
                        break;
												
                    } */
                }
            }
            break;
        }

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
							
							  memset(m_last_maclist,0,sizeof(m_last_maclist));
								memcpy(m_last_maclist,m_maclist,m_beacon_num * 7);	
							  m_last_beacon_num = m_beacon_num;	
							
                DEBUG_PRINT(0,"[APPL]: Scan timed out. cnt=%d\r\n",m_beacon_num);
              //  scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                DEBUG_PRINT(0,"[APPL]: Connection Request timed out.\r\n");
            }
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
		//DEBUG_PRINT(0,"evt scan callback\r\n");

    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    app_rfid_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
/*
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_8000MS_CALIBRATION, NULL);

#if defined(S110) || defined(S130) || defined(S132)
    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if (defined(S130) || defined(S132))
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
#endif
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}
*/
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    //SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);
	 SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_8000MS_CALIBRATION, NULL);
	
    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
	ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;  ////不太明白该属性
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling a radio RFID error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void radio_rfid_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

#if USE_WDT_MODULE  

/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void)
{
	DEBUG_PRINT(0, "wdt_event_handler\n");
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

static void wdt_config()
{
    uint32_t err_code = NRF_SUCCESS;
    //Configure WDT.
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
}
#endif //USE_WDT_MODULE

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

void radio_parm_check_and_update(radio_parm *p_data)
{
    ble_rfid_init_t rfid_init;
    radio_parm current_radio_parm=
    {
        current_version_id,//<版本
        18,                 //<信道f=2400MHz + RF_CH×1.0MHz
        RADIO_MODE_MODE_Nrf_250Kbit,    //<空中速率 0：1mbps ;2mbps; 250kbps。
        UESB_CRC_16BIT,      //<校验 0 ：CRC_OFF ；1：CRC_8BIT；2 ：CRC_16BIT。
        4,    //<地址长度 3：3 bytes； 4：4 bytes ; 5:5 bytes。
        {0x34, 0x12, 0xBB, 0xAA,0xFF}, //<发送地址
        32768U, // 休眠时间：1000ms
        8,
        {0x02,0X00,0xBC,0x61,0x11,0x00,0X33,0xff}

    };

   //支持版本检测
    if(p_data->version_id!=current_version_id)
    {
        goto radio_parm_check_and_update_end;
    }
	
	//flash没有写入数据的时候是0XFF,判断几个连续是0XFF作为判断依据
	if((p_data->rf_parm.rf_channel == 0XFF)&&(p_data->rf_parm.datarate == 0XFF)&&(p_data->rf_parm.crc_mode == 0XFF))
	{
		goto radio_parm_check_and_update_end;
	}

	/*还需要判断flash中写入数据是有效*/

	//radio支持的速率只有1Mbps、2Mbps、250Kbps三种
	if(p_data->rf_parm.datarate>UESB_BITRATE_250KBPS)
	{
		goto radio_parm_check_and_update_end;
	}

	//CRC支持的是OFF、CRC8、CRC16三种
	if(p_data->rf_parm.crc_mode>RADIO_CRCCNF_LEN_Two)
	{
		goto radio_parm_check_and_update_end;
	}

	//地址长度支持的是3、4、5的三种
	if((p_data->rf_parm.adder_with>5)||(p_data->rf_parm.adder_with<3))
	{
		goto radio_parm_check_and_update_end;
	}

	//ployload支持的最大长度为32
	if(p_data->pl_parm.ployload_len>32)
	{
		goto radio_parm_check_and_update_end;
	}

	memcpy(&current_radio_parm,p_data,sizeof(current_radio_parm));

radio_parm_check_and_update_end:

	//更新
	rfid_init.bitrate        = current_radio_parm.rf_parm.datarate;
	rfid_init.crc            = current_radio_parm.rf_parm.crc_mode; 
	rfid_init.rf_channel     = current_radio_parm.rf_parm.rf_channel;
	rfid_init.payload_length = current_radio_parm.pl_parm.ployload_len;
	rfid_init.rf_addr_length = current_radio_parm.rf_parm.adder_with;
	
	memcpy(rfid_init.rx_address_p0, current_radio_parm.rf_parm.tx_address, rfid_init.rf_addr_length);
	memcpy(rfid_init.ployload_buf,current_radio_parm.pl_parm.ployload_buf,rfid_init.payload_length);
    rfid_init.adv_interval  = RADIO_RFID_INTERVAL;
    rfid_init.error_handler = radio_rfid_error_handler;
    
    app_rfid_init(&rfid_init);

}

/**@brief Function for handling a BeaconScanner event.
 *
 * @details This function will be called each time the scanner has found an advertiser with beacon data.
 *          In this simple implementation, it checks the Major and Minor and company ID fields from the beacon data,
 *          and if they match, @ref BSP_INDICATE_ALERT_3 is indicated. 
 *
 * @param[in]   p_evt   scanner event data.
 */
static void beacon_evt_handler(ble_scan_beacon_evt_t * p_evt)
{
    uint32_t err_code;
		ble_gap_addr_t gap_addr = {0};	
	
	
//    if((p_evt->rcv_adv_packet.adv_data.major    == SEARCHED_MAJOR)
//     &&(p_evt->rcv_adv_packet.adv_data.minor    == SEARCHED_MINOR)
//     &&(p_evt->rcv_adv_packet.adv_data.manuf_id == APP_COMPANY_IDENTIFIER))
//    {
//        err_code = bsp_indication_set(BSP_INDICATE_ALERT_3);
//        APP_ERROR_CHECK(err_code);
//    }
		
		DEBUG_PRINT(0,"beacon_evt_handle\r\n");
		//memcpy(&gap_addr,&p_evt->rcv_adv_packet.addr,sizeof(gap_addr));
		//DEBUG_PRINT(0,"SCAN %02d:%02d:%02d:%02d:%02d:%02d\r\n",gap_addr.addr[0],gap_addr.addr[1],gap_addr.addr[2],gap_addr.addr[3],gap_addr.addr[4],gap_addr.addr[5]);
		
}


/**@brief Function for handling a BeaconScanner error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void beacon_scanner_error_handler(uint32_t nrf_error)
{
		DEBUG_PRINT(0,"beacon_error_handle\r\n");

    APP_ERROR_HANDLER(nrf_error);
}

static ble_beacon_scanner_init_t m_beacon_scanner_init;
/*end addition for beacon*/

///test
/**@brief Function for initializing the beacon scanner module.
 */
static void scanner_init()
{
    m_beacon_scanner_init.evt_handler   = beacon_evt_handler;
    m_beacon_scanner_init.error_handler = beacon_scanner_error_handler;
    app_beacon_scanner_init(&m_beacon_scanner_init);
}


/**@brief Function for starting the beacon scanner module.
 */
static void scanner_start()
{
    app_beacon_scanner_start();
}

//beacon  houqz test
//#include "advertiser_ibeacon.h"
#define IS_SRVC_CHANGED_CHARACT_PRESENT      0                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define APP_COMPANY_IDENTIFIER               0x0059                                     /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */

#define BEACON_UUID 0xff, 0xfe, 0x2d, 0x12, 0x1e, 0x4b, 0x0f, 0xa4,\
                    0x99, 0x4e, 0xce, 0xb5, 0x31, 0xf4, 0x05, 0x45 
#define BEACON_ADV_INTERVAL                  400                                        /**< The Beacon's advertising interval, in milliseconds*/
#define BEACON_MAJOR                         0x1234                                     /**< The Beacon's Major*/
#define BEACON_MINOR                         0x5678                                     /**< The Beacon's Minor*/
#define BEACON_RSSI                          0xC3                                       /**< The Beacon's measured RSSI at 1 meter distance in dBm. */


static ble_beacon_init_t beacon_init;

static void beacon_adv_init(void)
{
    static uint8_t beacon_uuid[] = {BEACON_UUID};
    
   // memcpy(beacon_init.uuid.uuid128, beacon_uuid, sizeof(beacon_uuid));
    beacon_init.adv_interval  = BEACON_ADV_INTERVAL;
	
		memcpy(beacon_init.uuid.uuid128, m_beacon_save_data.manu_uuid, sizeof(beacon_uuid));
		
	//if(m_beacon_save_data.)
    beacon_init.major         = m_beacon_save_data.major;//BEACON_MAJOR;
    beacon_init.minor         = m_beacon_save_data.minjor;BEACON_MINOR;
    beacon_init.manuf_id      = APP_COMPANY_IDENTIFIER;
    beacon_init.rssi          = BEACON_RSSI;
    beacon_init.error_handler = NULL;
    
    uint32_t err_code = sd_ble_gap_address_get(&beacon_init.beacon_addr);
    APP_ERROR_CHECK(err_code);
    
    app_beacon_init(&beacon_init);
}

#define UICR_ADDRESS                    0x10001080                        /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
void test_write_uicr(void)
{
	////testhouqz WUICR
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

		DEBUG_PRINT(0,"maj0x%x,min0x%x\r\n",major_value,minor_value);
		
	///	(*(uint32_t *)UICR_ADDRESS) = 0x01020304;
	
	//		nrf_delay_ms(100);
	//		NVIC_SystemReset();			//重启复位,离开工厂模式和重新启用工厂模式时		

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
	 while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        *(uint32_t *)0x10001080 = 0x22223333 ;
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;	
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

			nrf_delay_ms(1000);
			NVIC_SystemReset();			//重启复位,离开工厂模式和重新启用工厂模式时		
					
}

///关闭串口 单纯的diable是否有效待定
///app_uart_close

void ble_wakeup_mtk_uart(void)
{
	   return;
		 if(m_ready_send_mac)
		 {
			 return;
		 }
		 
		 m_ready_send_mac = true;
		 
		 nrf_gpio_cfg(TX_PIN_NUMBER,
                 NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_NOSENSE);
		
		///写低电平
		nrf_gpio_pin_clear(TX_PIN_NUMBER);
		DEBUG_PRINT(0,"PULL DOWN\r\n");
}

void beacon_next_scan_timer_start(uint16_t sec)
{
		uint32_t err_code;
		uint32_t ticks = 0; //APP_TIMER_TICKS(21000,APP_TIMER_PRESCALER)
    // Start application timers.
    //err_code = app_timer_start(m_start_scan_beacon_timer_id, BEACON_NEXT_SCAN_INTERVAL, NULL);
	  
	  if(sec > 0)
		{
			ticks = APP_TIMER_TICKS(sec*1000,APP_TIMER_PRESCALER);
			err_code = app_timer_start(m_start_scan_beacon_timer_id, ticks, NULL);
			APP_ERROR_CHECK(err_code);
	  }
		else
		{
			err_code = app_timer_stop(m_start_scan_beacon_timer_id);
		}
	DEBUG_PRINT(0,"beacon scan timer start sec:%d,ticks:%d\r\n",sec,ticks);

}

//static void nvm_write_beacon_scan_sta_complete_cb(uint32_t result)
//{
//	uint8_t load_data[4];
//	if (result == NRF_SUCCESS)
//	{
//		DEBUG_PRINT(0,"pstorage beacon scan callback received \r\n");

//		nvm_load(BEACON_SCAN_STA,load_data,4,0); 
//		DEBUG_PRINT(0,"    wait update load_data: %02x", load_data[0]);
//		DEBUG_PRINT(0," %02x", load_data[1]);
//		DEBUG_PRINT(0," %02x", load_data[2]);
//		DEBUG_PRINT(0," %02x\n", load_data[3]);
//	//	nrf_delay_ms(100);
//	//	NVIC_SystemReset();			//重启复位,离开工厂模式和重新启用工厂模式时?
//	}
//	else
//	{
//		DEBUG_PRINT(0,"pstorage UPDATE beacon scan ERROR \r\n");
//	}

//}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;

    
   DEBUG_PRINT(0, "BY libra .\n");


    // Initialize.
    timers_init();
    ble_stack_init();
    
    scheduler_init();
	
		uart_init();
		gpiote_init();
		action_uart_button_init();		
		action_uart_disable();
	
		// Initialize persistent storage module.
		err_code = pstorage_init();
		APP_ERROR_CHECK(err_code);
		
		nvm_storage_init();

		nvm_load(ADV_DATA_BLOCK_ID,m_baby_id_info, 4, 0);
		nvm_load(RADIO_PARM_BLOCK_ID,m_radio_parm_info,APP_RADIO_PARM_INFO_LENGTH, 0);
//		nvm_load(BEACON_SAVE_DATA,(uint8_t*)&m_beacon_save_data,sizeof(m_beacon_save_data),0);
		nvm_load(BEACON_SCAN_STA,m_beacon_start_sta,4,0);
		
		DEBUG_PRINT(0,"beacon sta:%d,%d,%d,%d\r\n",m_beacon_start_sta[0],m_beacon_start_sta[1],m_beacon_start_sta[2],m_beacon_start_sta[3]);
		
//		if(m_beacon_start_sta[0] < 5)
//		{
//			m_beacon_start_sta[0] = 5;
//		}	
//		m_beacon_start_sta[0] = 1;
//			m_beacon_start_sta[1] = 2;
//		m_beacon_start_sta[2] = 3;
//		m_beacon_start_sta[3] = 4;
//	
//	  nvm_write(BEACON_SCAN_STA,m_beacon_start_sta,4,0,nvm_write_beacon_scan_sta_complete_cb); ///test

		//radio_parm_check_and_update((radio_parm*)&m_radio_parm_info);
    gap_params_init();
 
		beacon_adv_init();
 
		advertising_init();
    services_init();
    conn_params_init();
		scanner_init();
				
	//	test_write_uicr();

    // Start execution.
   application_timers_start();
	 // advertising_start();
		//app_beacon_start();    

//	app_rfid_start();
	 //scanner_start();

	//sd_power_system_off();	///无效

#ifdef USE_WDT_MODULE
    //Configure WDT.
    wdt_config(); 
#endif //USE_WDT_MODULE
    // Enter main loop. 
    for (;;)
    {       
        app_sched_execute();
        power_manage();
    }
			DEBUG_PRINT(0,"main loop\r\n");


}


