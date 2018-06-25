/**
 ****************************************************************************************
 *
 * @file user_config_sw_ver.h
 *
 * @brief Use these defines to tag the user code software version. These defines are used by the DIS service.
 *
 * Copyright (C) 2014. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */
 
#ifndef USER_CONFIG_SW_VER_H_
#define USER_CONFIG_SW_VER_H_

 



#define DEVICE_NAME                          "hqz"              /**< Name of device. Will be included in the advertising data. */

#define MANUFACTURER_NAME                    "ruitel"              /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUMBER 					     "8602M1"
#define HARDWARE_REVISION 				     "8602M1_A13"
#define FIRMWARE_REVISION 				     "SDK_A"

#define SVN_BUILD_NO                         "0245"
#define SOFTWARE_REVISION                    "A13v16.1026."SVN_BUILD_NO

#define WATCH_SW_VERSION                      "\x00"SOFTWARE_REVISION

#endif 
