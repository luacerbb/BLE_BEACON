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
 
#include "advertiser_beacon.h"
#include <stdio.h>
#include <string.h>

#include "nrf_soc.h"
#include "app_error.h"
#include "app_util.h"
#include "debug_utils.h"


#define RFID_SLOT_LENGTH             4500

static struct
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
	bool                    keep_running;                       /** */
    bool                    is_running;                         /** is the 'RFID' running*/
    uint32_t                slot_length;                        /** */
    nrf_radio_request_t     timeslot_request;                   /** */
}m_rfid;


// Function that swaps the bits within each byte in a uint32. Used to convert from nRF24L type addressing to nRF51 type addressing
static uint32_t bytewise_bit_swap(uint32_t inp)
{
    inp = (inp & 0xF0F0F0F0) >> 4 | (inp & 0x0F0F0F0F) << 4;
    inp = (inp & 0xCCCCCCCC) >> 2 | (inp & 0x33333333) << 2;
    return (inp & 0xAAAAAAAA) >> 1 | (inp & 0x55555555) << 1;
}



enum mode_t
{
  ADV_INIT,                                                 /** Initialisation*/
  ADV_RX_RFID, 
  ADV_DONE                                                  /** Done advertising*/
};


nrf_radio_request_t * m_configure_next_event(void)
{
    m_rfid.timeslot_request.request_type              = NRF_RADIO_REQ_TYPE_NORMAL;
    m_rfid.timeslot_request.params.normal.hfclk       = NRF_RADIO_HFCLK_CFG_FORCE_XTAL;
    m_rfid.timeslot_request.params.normal.priority    = NRF_RADIO_PRIORITY_HIGH;
    m_rfid.timeslot_request.params.normal.distance_us = m_rfid.adv_interval * 1000;
    m_rfid.timeslot_request.params.normal.length_us   = m_rfid.slot_length;
    return &m_rfid.timeslot_request;
}


uint32_t m_request_earliest(enum NRF_RADIO_PRIORITY priority)
{
    m_rfid.timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_rfid.timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_FORCE_XTAL;
    m_rfid.timeslot_request.params.earliest.priority    = priority;
    m_rfid.timeslot_request.params.earliest.length_us   = m_rfid.slot_length;
    m_rfid.timeslot_request.params.earliest.timeout_us  = 1000000;
    return sd_radio_request(&m_rfid.timeslot_request);
}

 
/** 
 * @brief Function for configuring the radio to operate in Shockburst compatible mode.
 * 
*/
static void m_configure_rfid()
{
	//DEBUG_PRINT(0, " m_configure_rfid\n");	
	
	NRF_RADIO->POWER        = 1;
	
	NRF_RADIO->FREQUENCY = m_rfid.rf_channel;
	
   // Packet configuration
	// S1 size = 0 bits
	// S0 size = 0 bytes
	// payload length size = 0 bits
    NRF_RADIO->PCNF0 = (0<< RADIO_PCNF0_S1LEN_Pos) |
                       (0<< RADIO_PCNF0_S0LEN_Pos) |
                       (0<< RADIO_PCNF0_LFLEN_Pos); 
					   
	
	// Packet configuration:
    // Bit 25: 1 Whitening enabled
    // Bit 24: 1 Big endian,
    // 4 byte base address length (5 byte full address length), 
    // 0 byte static length, max 255 byte payload .	
	NRF_RADIO->PCNF1        =   (((RADIO_PCNF1_ENDIAN_Big)                 << RADIO_PCNF1_ENDIAN_Pos    ) & RADIO_PCNF1_ENDIAN_Msk)
                              | (((m_rfid.rf_addr_length-1)        << RADIO_PCNF1_BALEN_Pos     ) & RADIO_PCNF1_BALEN_Msk)
                              | ((((uint32_t)m_rfid.payload_length) << RADIO_PCNF1_STATLEN_Pos   ) & RADIO_PCNF1_STATLEN_Msk)
                              | ((((uint32_t)m_rfid.payload_length)<< RADIO_PCNF1_MAXLEN_Pos    ) & RADIO_PCNF1_MAXLEN_Msk)
                              | ((RADIO_PCNF1_WHITEEN_Disabled             << RADIO_PCNF1_WHITEEN_Pos   ) & RADIO_PCNF1_WHITEEN_Msk);
    
	// CRC configuration
    NRF_RADIO->CRCCNF    = (((m_rfid.crc) << RADIO_CRCCNF_LEN_Pos) & RADIO_CRCCNF_LEN_Msk); 
    if(m_rfid.crc == RADIO_CRCCNF_LEN_Two)
    {
        NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value      
        NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
    }
    else if(m_rfid.crc == RADIO_CRCCNF_LEN_One)
    {
        NRF_RADIO->CRCINIT = 0xFFUL;        // Initial value
        NRF_RADIO->CRCPOLY = 0x107UL;       // CRC poly: x^8+x^2^x^1+1
    }
	   //接收地址设定
	NRF_RADIO->RXADDRESSES  = ((RADIO_RXADDRESSES_ADDR0_Enabled)  << RADIO_RXADDRESSES_ADDR0_Pos);
    NRF_RADIO->SHORTS       = ((1 << RADIO_SHORTS_READY_START_Pos) | (1 << RADIO_SHORTS_END_DISABLE_Pos));
    
	// RF bitrate
	NRF_RADIO->MODE         = ((m_rfid.bitrate  )        << RADIO_MODE_MODE_Pos       ) & RADIO_MODE_MODE_Msk;

	NRF_RADIO->TIFS         = 150;
    NRF_RADIO->INTENSET     = (1 << RADIO_INTENSET_DISABLED_Pos);
	
	// Radio address config
    NRF_RADIO->PREFIX0      = bytewise_bit_swap(m_rfid.rx_address_p0[0]); 
    NRF_RADIO->BASE0        = bytewise_bit_swap(m_rfid.rx_address_p0[1] << 24 | m_rfid.rx_address_p0[2] << 16 | m_rfid.rx_address_p0[3] << 8 | m_rfid.rx_address_p0[4]);  

	//DMA指针指向发送的BUFFER
    NRF_RADIO->PACKETPTR    = (uint32_t) m_rfid.ployload_buf;
    
    NVIC_EnableIRQ(RADIO_IRQn);
	
}

void m_handle_start(void)
{
	//DEBUG_PRINT(0, " m_handle_start\n");
    // Configure TX_EN on TIMER EVENT_0
    NRF_PPI->CH[8].TEP    = (uint32_t)(&NRF_RADIO->TASKS_TXEN);
    NRF_PPI->CH[8].EEP    = (uint32_t)(&NRF_TIMER0->EVENTS_COMPARE[0]);
    NRF_PPI->CHENSET      = (1 << 8);
    
    // Configure and initiate radio
    m_configure_rfid();
    NRF_RADIO->TASKS_DISABLE = 1;
}

void m_handle_radio_disabled(enum mode_t mode)
{
    switch (mode)
    {
        case ADV_RX_RFID:
			//DEBUG_PRINT(0, " ADV_RX_RFID\n");
            NRF_RADIO->TASKS_TXEN = 1;
            break;
        default:
            break;
    }
}

static nrf_radio_signal_callback_return_param_t * m_timeslot_callback(uint8_t signal_type)
{
  static nrf_radio_signal_callback_return_param_t signal_callback_return_param;
  static enum mode_t mode;

  signal_callback_return_param.params.request.p_next  = NULL;
  signal_callback_return_param.callback_action        = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;

  //DEBUG_PRINT(0, " nrf_radio_signal_callback_return_param_t[signal_type] 0x%04x\n", signal_type);
  
  switch (signal_type)
  {
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:

      m_handle_start();

      mode = ADV_INIT;
      mode++;
	  //DEBUG_PRINT(0, " mode 0x%#04x\n", mode);
      break;
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
      if (NRF_RADIO->EVENTS_DISABLED == 1)
      {
        NRF_RADIO->EVENTS_DISABLED = 0;

        m_handle_radio_disabled(mode);

        if (mode == ADV_DONE)
        {
            NRF_PPI->CHENCLR = (1 << 8);
            if (m_rfid.keep_running)
            {
                signal_callback_return_param.params.request.p_next = m_configure_next_event();
                signal_callback_return_param.callback_action       = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
            }
            else
            {
                signal_callback_return_param.callback_action       = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
            }
            break;
        }
        mode++;
		//DEBUG_PRINT(0, " mode 0x%#04x\n", mode);
      }
      break;
    default:
        if (m_rfid.error_handler != NULL)
        {
            m_rfid.error_handler(NRF_ERROR_INVALID_STATE);
        }
      break;
  }

  return ( &signal_callback_return_param );
}

void app_rfid_on_sys_evt(uint32_t event)
{
    uint32_t err_code;
	//DEBUG_PRINT(0, " app_rfid_on_sys_evt[event] 0x%#04x\n", event);
    switch (event)
    {
        case NRF_EVT_RADIO_SESSION_IDLE:
            if (m_rfid.is_running)
            {
                m_rfid.is_running = false;
                err_code = sd_radio_session_close();
                if ((err_code != NRF_SUCCESS) && (m_rfid.error_handler != NULL))
                {
                    m_rfid.error_handler(err_code);
                }
            }
            break;
        case NRF_EVT_RADIO_SESSION_CLOSED:
            break;
        case NRF_EVT_RADIO_BLOCKED:
        case NRF_EVT_RADIO_CANCELED: // Fall through
            if (m_rfid.keep_running)
            {
                // TODO: A proper solution should try again in <block_count> * m_rfid.adv_interval
                err_code = m_request_earliest(NRF_RADIO_PRIORITY_HIGH);
                if ((err_code != NRF_SUCCESS) && (m_rfid.error_handler != NULL))
                {
                    m_rfid.error_handler(err_code);
                }
            }
            break;
        default:
            break;
    }
}

void app_rfid_init(ble_rfid_init_t * p_init)
{
	//DEBUG_PRINT(0, " app_rfid_init\n");

	m_rfid.adv_interval   =  p_init->adv_interval;
    m_rfid.slot_length    =  RFID_SLOT_LENGTH;
    m_rfid.error_handler  =  p_init->error_handler;	 
    m_rfid.bitrate        =  p_init->bitrate;
	m_rfid.crc            =  p_init->crc; 
	m_rfid.rf_channel     =  p_init->rf_channel;
	m_rfid.payload_length =  p_init->payload_length;
	m_rfid.rf_addr_length =  p_init->rf_addr_length;
	
	memcpy(m_rfid.ployload_buf,p_init->ployload_buf,m_rfid.payload_length);
	memcpy(m_rfid.rx_address_p0,p_init->rx_address_p0, m_rfid.rf_addr_length);
}

void app_rfid_start(void)
{
    m_rfid.keep_running = true;
    m_rfid.is_running   = true;
	//DEBUG_PRINT(0, " app_rfid_start\n");
    uint32_t err_code = sd_radio_session_open(m_timeslot_callback);
    if ((err_code != NRF_SUCCESS) && (m_rfid.error_handler != NULL))
    {
        m_rfid.error_handler(err_code);
    }
    
    err_code = m_request_earliest(NRF_RADIO_PRIORITY_NORMAL);
    if ((err_code != NRF_SUCCESS) && (m_rfid.error_handler != NULL))
    {
        m_rfid.error_handler(err_code);
    }
}

void app_rfid_stop(void)
{
	//DEBUG_PRINT(0, "app_rfid_stop\n");
    m_rfid.keep_running = false;
}


