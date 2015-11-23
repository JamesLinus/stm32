#ifndef MQUEUE_H__
#define MQUEUE_H__
#include "time.h"
#include <stddef.h>
#include "fcntl.h"

#include "FreeRTOS.h"
#include "queue.h"

#define mqd_t	QueueHandle_t
/* On success, mq_open() returns a message queue descriptor for use by other message queue functions.  On error, mq_open() returns (mqd_t) -1, with errno set to indicate the error.
 * oflag = queue len
 */ 
mqd_t mq_open(const char *name, int oflag);
//On success mq_close() returns 0; on error, -1 is returned, with errno set to indicate the error.
int mq_close(mqd_t mqdes);
//On success, mq_receive() and mq_timedreceive() return the number of bytes in the received message; on error, -1 is returned, with errno set to indicate the error.
int mq_timedreceive(mqd_t mqdes, char *msg_ptr, size_t msg_len, unsigned *msg_prio, const struct timespec *abs_timeout);
//On success, mq_send() and mq_timedsend() return zero; on error, -1 is returned, with errno set to indicate the error.
int mq_send(mqd_t mqdes, const char *msg_ptr, size_t msg_len, unsigned msg_prio);
#endif
