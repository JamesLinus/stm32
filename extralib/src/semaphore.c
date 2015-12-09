#include "../include/semaphore.h"
#include "drv_api.h"
#include <lib_defines.h>
//sem_init() returns 0 on success; on error, -1 is returned, and errno is set to indicate the error.
int sem_init(sem_t *sem, int pshared, unsigned int value){
#if defined(OS_FREERTOS)
	*sem = xSemaphoreCreateCounting(SEM_MAX_COUNT, value);
	if(*sem) return 0;
#elif defined(OS_UCOS)
	OS_ERR err;
	OSSemCreate(sem, "", value, &err);
	if(OS_ERR_NONE == err) return 0;
#endif
	return -1;
}
//sem_destroy() returns 0 on success; on error, -1 is returned, and errno is set to indicate the error. 
int sem_destroy(sem_t *sem){
#if defined(OS_FREERTOS)
	vSemaphoreDelete(*sem);
#elif defined(OS_UCOS)
	OS_ERR err;
	OSSemDel(sem, OS_OPT_DEL_ALWAYS, &err);
#endif
	return 0;
}
//sem_post() returns 0 on success; on error, the value of the semaphore is left unchanged, -1 is returned, and errno is set to indicate the error.
int sem_post(sem_t *sem){
#if defined(OS_FREERTOS)
	if(xSemaphoreGive(*sem) == pdTRUE)
		return 0;
#elif defined(OS_UCOS)
	OS_ERR err;
	OSSemPost(sem, OS_OPT_POST_ALL, &err);
	if(err == OS_ERR_NONE) return 0;
#endif
	return -1;
}
//All of these functions (wait) return 0 on success; on error, the value of the semaphore is left unchanged, -1 is returned, and errno is set to indicate the error.
int sem_wait(sem_t *sem){
#if defined(OS_FREERTOS)
	if(xSemaphoreTake(*sem, portMAX_DELAY) == pdTRUE) return 0;
#elif defined(OS_UCOS)
	OS_ERR err;
	OSSemPend(sem, 0, OS_OPT_PEND_BLOCKING, 0, &err);
	if(err == OS_ERR_NONE) return 0;
#endif
	return -1;
}
int sem_timedwait(sem_t *sem, const struct timespec *abs_timeout){
#if defined(OS_FREERTOS)
	TickType_t timeout = abs_timeout->tv_sec * 1000 / portTICK_PERIOD_MS;
	timeout += abs_timeout->tv_nsec / 1000000 / portTICK_PERIOD_MS;
	if(xSemaphoreTake(*sem, timeout) == pdTRUE) return 0;
	errno = ETIMEDOUT;
#elif defined(OS_UCOS)
	OS_ERR err;
	OS_TICK timeout = abs_timeout->tv_sec * 1000 * TICK_RATE_HZ;
	timeout += abs_timeout->tv_nsec / 1000000 * TICK_RATE_HZ;
	OSSemPend(sem, timeout, OS_OPT_PEND_BLOCKING, 0, &err);
	if(err == OS_ERR_NONE) return 0;
#endif
	return -1;
}
// end of file
