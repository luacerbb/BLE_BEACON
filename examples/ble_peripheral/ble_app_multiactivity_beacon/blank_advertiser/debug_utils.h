#ifndef __DEBUG_UTILS_H__
#define __DEBUG_UTILS_H__

#include "nrf51.h"
#include "nrf51_bitfields.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "SEGGER_RTT.h"


#if defined(DEBUG) || defined(DEBUG_VERBOSE)
#define DEBUG_PRINT  SEGGER_RTT_printf
#else
#define DEBUG_PRINT(...)
#endif /* defined(DEBUG) || defined(DEBUG_VERBOSE) */

#if defined(DEBUG_VERBOSE)
#define VERBOSE_PRINT printf
#else
#define VERBOSE_PRINT(...)
#endif

#endif /* __DEBUG_UTILS_H__ */
