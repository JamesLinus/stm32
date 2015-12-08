#include  <cpu.h>
#include  <lib_math.h>
#include  <lib_mem.h>
#include  <os.h>
#include  <os_app_hooks.h>
#include  <app_cfg.h>
#include  <bsp.h>

#include <pthread.h>
#include <debug.h>

int					g_fd_led[4] 		= {-1};
int                 g_fd_button         = -1;

#define             APP_THREAD_COUNT    			1
pthread_t           g_thread[APP_THREAD_COUNT];
pthread_attr_t      g_thread_attr[APP_THREAD_COUNT];
int g_thread_index = 1;
#define DEFINE_THREAD(fxn, stack_size, priority) {\
    pthread_attr_setstacksize(&g_thread_attr[g_thread_index], stack_size);\
    pthread_setschedprio(&g_thread[g_thread_index], priority);\
    pthread_create(&g_thread[g_thread_index], &g_thread_attr[g_thread_index], fxn, &g_thread_startup[g_thread_index-1]);\
    g_thread_index++;\
}

void *Thread_Startup(void*);

int __errno;
int main(void)
{
    OS_ERR   err;
    HAL_Init();                                                 /* See Note 1.                                          */
    Mem_Init();                                                 /* Initialize Memory Managment Module                   */
    Math_Init();                                                /* Initialize Mathematical Module                       */
    BSP_IntDisAll();                                            /* Disable all Interrupts.                              */

    OSInit(&err);                                               /* Init uC/OS-III.                                      */
    App_OS_SetAllHooks();

    pthread_attr_setstacksize(&g_thread_attr[0], 1024*3);
    pthread_setschedprio(&g_thread[0], 1);
    pthread_create(&g_thread[0], &g_thread_attr[0], Thread_Startup, 0);

    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */

    while (DEF_ON) {}
}

void  *Thread_Startup (void *p_arg)
{
    OS_ERR      err;
    BSP_Init();                                                 /* Initialize BSP functions                             */
    CPU_Init();                                                 /* Initialize the uC/CPU services                       */

    driver_probe();
    board_register_devices();

    g_fd_led[0] = open_dev("led-green", 0);
    g_fd_led[1] = open_dev("led-red", 	0);
    g_fd_led[2] = open_dev("led-blue", 0);
    g_fd_led[3] = open_dev("led-orange", 0);
    g_fd_button = open_dev("button", 	0);

    LED_ON(ORANGE);

    while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */
        LED_TOGGLE(ORANGE);
        OSTimeDlyHMSM(0u, 0u, 1u, 0u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
    }
}
// end of file
