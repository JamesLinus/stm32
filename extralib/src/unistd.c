#include "unistd.h"
#include "lib_defines.h"
#include "FreeRTOS.h"
#include "task.h"
unsigned int 	sleep (unsigned int __seconds){
	vTaskDelay(__seconds*1000 / portTICK_RATE_MS);
	return 0;
}
int 			usleep_s (unsigned int __useconds){
	vTaskDelay((__useconds > 1000) ? __useconds / 1000 / portTICK_RATE_MS : 1);
	return 0;
}
int clock_gettime(clockid_t clk_id, struct timespec *tp){
	int ret = 0;
	TickType_t tick = xTaskGetTickCount();

	tp->tv_sec = tick / 1000;
	tp->tv_nsec = (tick % 1000) * 1000000;

	return ret;
}
//end of file
