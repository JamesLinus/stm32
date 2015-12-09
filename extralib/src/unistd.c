#include <unistd.h>
#include <lib_defines.h>
#if defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
unsigned int 	sleep (unsigned int __seconds){
	vTaskDelay(__seconds*1000 * portTICK_RATE_MS);
	return 0;
}
int 			usleep_s (unsigned int __useconds){
	vTaskDelay((__useconds > 1000) ? __useconds * 1000 / portTICK_RATE_MS : 1);
	return 0;
}
int clock_gettime(clockid_t clk_id, struct timespec *tp){
	int ret = 0;
	TickType_t tick = xTaskGetTickCount();

	tp->tv_sec = tick / 1000;
	tp->tv_nsec = (tick % 1000) * 1000000;

	return ret;
}
#elif defined(OS_UCOS)
unsigned int 	sleep (unsigned int __seconds){
	OS_ERR err;
	if(__seconds <= 0) __seconds = 1;
	OSTimeDly(__seconds * TICK_RATE_HZ, OS_OPT_TIME_DLY, &err);
	return 0;
}
int 			usleep_s (unsigned int __useconds){
	OS_ERR err;
	OSTimeDly((__useconds > 1000) ? __useconds / 1000 * TICK_RATE_HZ / 1000 : 1, OS_OPT_TIME_DLY, &err);
	return 0;
}
int 			nanosleep(const struct timespec *req, struct timespec *rem){
	OS_TICK dly;
	OS_ERR err;
	dly = req->tv_sec * 1000 + req->tv_nsec / 1000000;
	if(dly <= 0) dly = 1;
	dly = dly * TICK_RATE_HZ / 1000;
	OSTimeDly(dly, OS_OPT_TIME_DLY, &err);
	return 0;
}
int clock_gettime(clockid_t clk_id, struct timespec *tp){
	int ret = 0;
	OS_ERR err;
	OS_TICK tick = OSTimeGet(&err);

	tp->tv_sec = tick / TICK_RATE_HZ;
	tp->tv_nsec = (tick % TICK_RATE_HZ) * 1000000;

	return ret;
}
#endif
//end of file
