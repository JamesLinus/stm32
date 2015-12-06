#include "pthread.h"

// end of file
int pthread_create (pthread_t * __newthread,
			   const pthread_attr_t * __attr,
			   void *(*__start_routine) (void *),
			   void * __arg){
	__newthread->attr = (pthread_attr_t*)__attr;
	__newthread->start_routine = __start_routine;
	__newthread->arg = __arg;

	  xTaskCreate(
			  (void (*)(void*))__newthread->start_routine,/* Function pointer */
			  "",                          				  /* Task name - for debugging only*/
			  __newthread->attr ?
					  __newthread->attr->stack_size
					  : configMINIMAL_STACK_SIZE,         /* Stack depth in words */
			  (void*) __newthread->arg,                   /* Pointer to tasks arguments (parameter) */
			  __newthread->prio,           			      /* Task priority*/
			  &__newthread->handle         				  /* Task handle */
	  );
	  return 0;
}
int pthread_attr_getstackaddr (const pthread_attr_t *
				      __attr, void ** __stackaddr){
	*__stackaddr = __attr->stack;
	return 0;
}
int pthread_attr_setstackaddr (pthread_attr_t *__attr,
				      void *__stackaddr){
	__attr->stack = __stackaddr;
	return 0;
}
int pthread_attr_getstacksize (const pthread_attr_t *
				      __attr, size_t * __stacksize){
	*__stacksize = __attr->stack_size;
	return 0;
}
int pthread_attr_setstacksize (pthread_attr_t *__attr,
				      size_t __stacksize){
	__attr->stack_size = __stacksize;
	return 0;
}
int pthread_setschedprio (pthread_t *__target_thread, int __prio){
	__target_thread->prio = __prio;
	return 0;
}
