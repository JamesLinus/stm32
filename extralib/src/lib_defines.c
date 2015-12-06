/*
 * lib_defs.c
 *
 *  Created on: Dec 5, 2015
 *      Author: dev
 */

#include <lib_defines.h>
void _lib_memcpy(void* dest, const void* src, int len){
	uint8_t *pd = (uint8_t*)dest;
	uint8_t *ps = (uint8_t*)src;
	while(len > 0){
		*pd = *ps;
		pd++;
		ps++;
		len --;
	}
}


