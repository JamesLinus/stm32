/*
 * flag_event.h
 *
 *  Created on: Nov 28, 2015
 *      Author: dev
 */

#ifndef INCLUDE_FLAG_EVENT_H_
#define INCLUDE_FLAG_EVENT_H_
#include "time.h"
#if defined(OS_FREERTOS)
#define flag_event_t	void*
#elif defined(OS_UCOS)
#include <os.h>
#define flag_event_t	OS_FLAG_GRP
#endif
int flag_event_init(flag_event_t* event);
int flag_event_destroy(flag_event_t *event);
int flag_event_post(flag_event_t *event);
int flag_event_wait(flag_event_t *event);
int flag_event_timedwait(flag_event_t *event, const struct timespec *abs_timeout);


#endif /* INCLUDE_FLAG_EVENT_H_ */
