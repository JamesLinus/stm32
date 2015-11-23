#ifndef SEMAPHORE_H__
#define SEMAPHORE_H__
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "time.h"
#define sem_t SemaphoreHandle_t
#define SEM_MAX_COUNT		999
//sem_init() returns 0 on success; on error, -1 is returned, and errno is set to indicate the error.
int sem_init(sem_t *sem, int pshared, unsigned int value);
//sem_destroy() returns 0 on success; on error, -1 is returned, and errno is set to indicate the error. 
int sem_destroy(sem_t *sem);
//sem_post() returns 0 on success; on error, the value of the semaphore is left unchanged, -1 is returned, and errno is set to indicate the error.
// do not call from iSR
int sem_post(sem_t *sem);
//All of these functions (wait) return 0 on success; on error, the value of the semaphore is left unchanged, -1 is returned, and errno is set to indicate the error.
// do not call from iSR
int sem_wait(sem_t *sem);
int sem_timedwait(sem_t *sem, const struct timespec *abs_timeout);

#endif
