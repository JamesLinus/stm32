/*
 * flag_event.c
 *
 *  Created on: Nov 28, 2015
 *      Author: dev
 */
#include "../include/flag_event.h"
#if defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#elif defined(OS_UCOS)
#include <os.h>
#include <lib_defines.h>
#endif
int flag_event_init(flag_event_t* event){
#if defined(OS_FREERTOS)
	*event = xEventGroupCreate();
	return (*event) ? 0 : -1;
#elif defined(OS_UCOS)
	OS_ERR err;
	OSFlagCreate(event, "", 0, &err);
	return (err == OS_ERR_NONE) ? 0 : -1;
#endif
}
int flag_event_destroy(flag_event_t *event){
	return -1;
}
int flag_event_post(flag_event_t *event){
	/* Make sure that interrupt flag is set */
#if defined(OS_FREERTOS)
	xEventGroupSetBits(
			*event,
			(((uint8_t)1)<< 0));
#elif defined(OS_UCOS)
	OS_ERR err;
	OSFlagPost(event, 1, OS_OPT_POST_FLAG_SET, &err);
#endif
	return 0;
}
int flag_event_wait(flag_event_t *event){
#if defined(OS_FREERTOS)
	EventBits_t uxBits;
	uxBits = xEventGroupWaitBits(*event,
			((uint32_t)1) << 0,
			pdTRUE,
			pdFALSE,
			portMAX_DELAY);
	if(uxBits & (((uint32_t)1) << 0))
		return 1;
	return 0;
#elif defined(OS_UCOS)
	OS_ERR err;
	OS_FLAGS flags;
	flags = OSFlagPend(event, 1, 0, OS_OPT_PEND_FLAG_SET_ANY, 0, &err);
	if((err == OS_ERR_NONE) && (flags & ((OS_FLAGS)1) == 1)) return 1;
	return 0;
#endif
}
int flag_event_timedwait(flag_event_t *event, const struct timespec *abs_timeout){
#if defined(OS_FREERTOS)
	EventBits_t uxBits;
	TickType_t timeout = abs_timeout->tv_sec * 1000 * portTICK_PERIOD_MS;
	timeout += abs_timeout->tv_nsec / 1000000 * portTICK_PERIOD_MS;
	uxBits = xEventGroupWaitBits(*event,
			((uint32_t)1) << 0,
			pdTRUE,
			pdFALSE,
			timeout);
	if(uxBits & (((uint32_t)1) << 0))
		return 1;
	return 0;
#elif defined(OS_UCOS)
	OS_ERR err;
	OS_FLAGS flags;
	OS_TICK timeout = abs_timeout->tv_sec * 1000 * TICK_RATE_HZ;
	timeout += abs_timeout->tv_nsec / 1000000 * TICK_RATE_HZ;
	flags = OSFlagPend(event, 1, timeout, OS_OPT_PEND_FLAG_SET_ANY, 0, &err);
	if((err == OS_ERR_NONE) && (flags & ((OS_FLAGS)1) == 1)) return 1;
	return 0;
#endif
}


// end of file
