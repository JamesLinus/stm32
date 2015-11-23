#include "../include/semaphore.h"
#include "drv_api.h"
//sem_init() returns 0 on success; on error, -1 is returned, and errno is set to indicate the error.
int sem_init(sem_t *sem, int pshared, unsigned int value){
	*sem = xSemaphoreCreateCounting(SEM_MAX_COUNT, value);
	if(*sem) return 0;
	return -1;
}
//sem_destroy() returns 0 on success; on error, -1 is returned, and errno is set to indicate the error. 
int sem_destroy(sem_t *sem){
	vSemaphoreDelete(*sem);
	return 0;
}
//sem_post() returns 0 on success; on error, the value of the semaphore is left unchanged, -1 is returned, and errno is set to indicate the error.
int sem_post(sem_t *sem){
	if(xSemaphoreGive(*sem) == pdTRUE)
		return 0;
	return -1;
}
//All of these functions (wait) return 0 on success; on error, the value of the semaphore is left unchanged, -1 is returned, and errno is set to indicate the error.
int sem_wait(sem_t *sem){
	if(xSemaphoreTake(*sem, portMAX_DELAY) == pdTRUE) return 0;
	return -1;
}
int sem_timedwait(sem_t *sem, const struct timespec *abs_timeout){
	TickType_t timeout = abs_timeout->tv_sec * 1000 / portTICK_PERIOD_MS;
	timeout += abs_timeout->tv_nsec / 1000000 / portTICK_PERIOD_MS;
	if(xSemaphoreTake(*sem, timeout) == pdTRUE) return 0;
	errno = ETIMEDOUT;
	return -1;
}
// end of file
