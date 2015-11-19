#include "../include/mqueue.h"
//On success, mq_open() returns a message queue descriptor for use by other message queue functions.  On error, mq_open() returns (mqd_t) -1, with errno set to indicate the error.
mqd_t mq_open(const char *name, int oflag){
	mqd_t ret = xQueueCreate(oflag, 1);
	return ret;
}
//On success mq_close() returns 0; on error, -1 is returned, with errno set to indicate the error.
int mq_close(mqd_t mqdes){
	vQueueDelete(mqdes);
	return 0;
}
//On success, mq_receive() and mq_timedreceive() return the number of bytes in the received message; on error, -1 is returned, with errno set to indicate the error.
ssize_t mq_timedreceive(mqd_t mqdes, char *msg_ptr, size_t msg_len, unsigned *msg_prio, const struct timespec *abs_timeout){
	ssize_t ret = 0;
	TickType_t timeout = abs_timeout->tv_sec * 1000 / portTICK_PERIOD_MS;
	timeout += abs_timeout->tv_nsec / 1000000 / portTICK_PERIOD_MS;
	while(msg_len){
		if(xQueueReceive(mqdes, msg_ptr, timeout) != pdTRUE){
			break;
		}
		msg_ptr++;
		ret++;
		msg_len--;
	}
	return ret;
}
//On success, mq_send() and mq_timedsend() return zero; on error, -1 is returned, with errno set to indicate the error.
int mq_send(mqd_t mqdes, const char *msg_ptr, size_t msg_len, unsigned msg_prio){
	while(msg_len > 0){
		if(xQueueSend(mqdes, msg_ptr, portMAX_DELAY) != pdTRUE) break;
		msg_len --;
		msg_ptr++;
	}
	return 0;
}
// end of file
