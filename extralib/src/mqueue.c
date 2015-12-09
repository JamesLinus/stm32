#include "../include/mqueue.h"
//On success, mq_open() returns a message queue descriptor for use by other message queue functions.  On error, mq_open() returns (mqd_t) -1, with errno set to indicate the error.
mqd_t mq_open(const char *name, int oflag){
#if defined(OS_FREERTOS)
	mqd_t ret = xQueueCreate(oflag, 1);
#elif defined(OS_UCOS)
	OS_Q *q;
	OS_ERR err;
	LIB_ERR l_err;
	q = (OS_Q*)Mem_SegAlloc(0, 0, sizeof(OS_Q), &l_err);
	OSQCreate(q, "", oflag, &err);
#endif
	return q;
}
//On success mq_close() returns 0; on error, -1 is returned, with errno set to indicate the error.
int mq_close(mqd_t mqdes){
#if defined(OS_FREERTOS)
	vQueueDelete(mqdes);
#elif defined(OS_UCOS)

#endif
	return 0;
}
//On success, mq_receive() and mq_timedreceive() return the number of bytes in the received message; on error, -1 is returned, with errno set to indicate the error.
int mq_timedreceive(mqd_t mqdes, char *msg_ptr, size_t msg_len, unsigned *msg_prio, const struct timespec *abs_timeout){
	int ret = 0;
#if defined(OS_FREERTOS)
	TickType_t timeout = abs_timeout->tv_sec * 1000 * portTICK_PERIOD_MS;
	timeout += abs_timeout->tv_nsec / 1000000 * portTICK_PERIOD_MS;
	while(msg_len){
		if(xQueueReceive(mqdes, msg_ptr, timeout) != pdTRUE){
			break;
		}
		msg_ptr++;
		ret++;
		msg_len--;
	}
#elif defined(OS_UCOS)
	uint8_t *pu8val;
	OS_ERR err;
	OS_MSG_SIZE p_msg_size;
	OS_TICK timeout = abs_timeout->tv_sec * 1000 * TICK_RATE_HZ;
	timeout += abs_timeout->tv_nsec / 1000000 * TICK_RATE_HZ;
	p_msg_size = 1;
	while(msg_len){
		pu8val = OSQPend(mqdes, timeout, OS_OPT_PEND_BLOCKING, &p_msg_size, 0, &err);
		if(err == OS_ERR_NONE){
			*msg_ptr = *pu8val;
		}else break;
		msg_ptr++;
		ret++;
		msg_len--;
	}
#endif
	return ret;
}
//On success, mq_send() and mq_timedsend() return zero; on error, -1 is returned, with errno set to indicate the error.
int mq_send(mqd_t mqdes, const char *msg_ptr, size_t msg_len, unsigned msg_prio){
#if defined(OS_UCOS)
	OS_ERR err;
#endif
	while(msg_len > 0){
#if defined(OS_FREERTOS)
		if(xQueueSend(mqdes, msg_ptr, portMAX_DELAY) != pdTRUE) break;
#elif defined(OS_UCOS)
		OSQPost(mqdes, (void*)msg_ptr, 1, OS_OPT_POST_ALL | OS_OPT_POST_FIFO, &err);
		if(err != OS_ERR_NONE) break;
#endif
		msg_len --;
		msg_ptr++;
	}
	return 0;
}
// end of file
