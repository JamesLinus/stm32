#ifndef LIB_DEFINES_H__
#define LIB_DEFINES_H__
#include <stdint.h>
#include <stddef.h>
// BSP
#if defined(STDPERIPH_DRIVER)
#elif defined(STM32CUBEF4)
#endif

// OS
#if defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#define TICK_RATE_HZ						portTICK_PERIOD_MS
#elif defined(OS_UCOS)
#include <os.h>
#include <os_cfg_app.h>
#define TICK_RATE_HZ 						OS_CFG_TICK_RATE_HZ
#endif


#define LIB_STRING_MAX_LENGTH	1024

void _lib_memcpy(void* dest, const void* src, int len);
#endif
