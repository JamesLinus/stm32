/*
 * App.c
 *
 *  Created on: Nov 28, 2015
 *      Author: dev
 */

#include "App.h"
#include "debug.h"
#include <semaphore.h>
#include <flag_event.h>

extern volatile uint8_t	g_debug_cmd;

int	App_Initialize(){
	return 0;
}
int  App_ProcessMessage(void* vmsg){
	int ret = 0;
	return ret;
}
int  App_SendMessage(void* msg, int len){
	int ret = 0;
	return ret;
}
// end of file
