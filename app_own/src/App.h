/*
 * App.h
 *
 *  Created on: Nov 28, 2015
 *      Author: dev
 */

#ifndef SRC_APP_H_
#define SRC_APP_H_
#include <stddef.h>
#include <stdint.h>

struct app_packet{
	uint8_t	payload[1];
};

int	App_Initialize();
int App_ProcessMessage(void* msg);
int App_SendMessage(void* msg, int len);

#endif /* SRC_APP_H_ */
