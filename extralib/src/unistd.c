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
//end of file
