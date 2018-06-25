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
#ifndef Watch_8602M1_H
#define Watch_8602M1_H

#include "nrf_gpio.h"

#include "app_gpiote.h"

#include "app_button.h"

#define RX_PIN_NUMBER  3
#define TX_PIN_NUMBER  2
#define CTS_PIN_NUMBER 0xFFFFFFFF    /**< Value indicating that no pin is connected to this UART register. */
#define RTS_PIN_NUMBER 0xFFFFFFFF    /**< Value indicating that no pin is connected to this UART register. */
#define HWFC           false
    
#define PIN_BUTTON_UART                  11
#define BUTTON_UART_PULL                 NRF_GPIO_PIN_NOPULL
#define BUTTON_UART_ACTIVE_STATE         APP_BUTTON_ACTIVE_LOW

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM


#endif // Watch_8602M1_H
