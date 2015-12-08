#ifndef PTHREAD_H
#define	PTHREAD_H
#include "lib_defines.h"
#if defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#elif defined(OS_UCOS)
#include  <os.h>
#include  <os_app_hooks.h>
#endif
typedef struct
{
    void*         stack;
    size_t		  stack_size;
}pthread_attr_t;
typedef struct {
    pthread_attr_t *attr;
    void *(*start_routine) (void *);
    void *arg;
    unsigned char prio;
#if defined(OS_FREERTOS)
    TaskHandle_t handle;
#elif defined(OS_UCOS)
    OS_TCB		handle;
#endif
}pthread_t;

/* Create a new thread, starting with execution of START-ROUTINE
   getting passed ARG.  Creation attributed come from ATTR.  The new
   handle is stored in *NEWTHREAD.  */
int pthread_create (pthread_t * __newthread,
			   const pthread_attr_t * __attr,
			   void *(*__start_routine) (void *),
			   void * __arg);

/* Terminate calling thread.

   The registered cleanup handlers are called via exception handling
   so we cannot mark this function with __THROW.*/
// void pthread_exit (void *__retval);

/* Make calling thread wait for termination of the thread TH.  The
   exit status of the thread is stored in *THREAD_RETURN, if THREAD_RETURN
   is not NULL.

   This function is a cancellation point and therefore not marked with
   __THROW.  */
// int pthread_join (pthread_t __th, void **__thread_return);

/* Indicate that the thread TH is never to be joined with PTHREAD_JOIN.
   The resources of TH will therefore be freed immediately when it
   terminates, instead of waiting for another thread to perform PTHREAD_JOIN
   on it.  */
// int pthread_detach (pthread_t __th);


/* Obtain the identifier of the current thread.  */
// pthread_t pthread_self (void);

/* Compare two thread identifiers.  */
// int pthread_equal (pthread_t __thread1, pthread_t __thread2);


/* Return the previously set address for the stack.  */
int pthread_attr_getstackaddr (const pthread_attr_t *
				      __attr, void ** __stackaddr);

/* Set the starting address of the stack of the thread to be created.
   Depending on whether the stack grows up or down the value must either
   be higher or lower than all the address in the memory block.  The
   minimal size of the block must be PTHREAD_STACK_MIN.  */
int pthread_attr_setstackaddr (pthread_attr_t *__attr,
				      void *__stackaddr);

/* Return the currently used minimal stack size.  */
int pthread_attr_getstacksize (const pthread_attr_t *
				      __attr, size_t * __stacksize);

/* Add information about the minimum stack size needed for the thread
   to be started.  This size must never be less than PTHREAD_STACK_MIN
   and must also not exceed the system limits.  */
int pthread_attr_setstacksize (pthread_attr_t *__attr,
				      size_t __stacksize);

/* Set the scheduling priority for TARGET_THREAD.  */
int pthread_setschedprio (pthread_t *__target_thread, int __prio);

#endif	/* PTHREAD_H */

