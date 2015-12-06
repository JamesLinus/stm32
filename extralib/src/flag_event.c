/*
 * flag_event.c
 *
 *  Created on: Nov 28, 2015
 *      Author: dev
 */
#include "../include/flag_event.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
int flag_event_init(flag_event_t* event){
	*event = xEventGroupCreate();
	return (*event) ? 0 : -1;
}
int flag_event_destroy(flag_event_t *event){
	return -1;
}
int flag_event_post(flag_event_t *event){
	/* Make sure that interrupt flag is set */
	xEventGroupSetBits(
			*event,
			(((uint8_t)1)<< 0));
	return 0;
}
int flag_event_wait(flag_event_t *event){
	EventBits_t uxBits;
	uxBits = xEventGroupWaitBits(*event,
			((uint32_t)1) << 0,
			pdTRUE,
			pdFALSE,
			portMAX_DELAY);
	if(uxBits & (((uint32_t)1) << 0))
		return 1;
	return 0;
}
int flag_event_timedwait(flag_event_t *event, const struct timespec *abs_timeout){
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
}


// end of file
