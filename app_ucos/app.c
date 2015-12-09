#include  <cpu.h>
#include  <lib_math.h>
#include  <lib_mem.h>
#include  <os.h>
#include  <os_app_hooks.h>
#include  <app_cfg.h>
#include  <bsp.h>

#include <debug.h>

#include <drv_api.h>
#include <pthread.h>
#include <unistd.h>
#include <termios.h>
#include <semaphore.h>
#include <mqueue.h>
#include <spidev.h>
#include <string.h>
#include <drv_gpio.h>

#include <sys/reboot.h>
int					g_fd_led[4] 		= {-1};
int                 g_fd_button         = -1;
int                 g_fd_debug          = -1;

uint8_t				g_debug_cmd			= 0;

#define             APP_THREAD_COUNT    			4
pthread_t           g_thread[APP_THREAD_COUNT];
pthread_attr_t      g_thread_attr[APP_THREAD_COUNT];
sem_t               g_thread_startup[APP_THREAD_COUNT-1];
int 				g_thread_index = 1;
sem_t				g_sem_debug;
mqd_t               g_debug_tx_buffer;

#define DEFINE_THREAD(fxn, stack_size, priority) {\
    pthread_attr_setstacksize(&g_thread_attr[g_thread_index], stack_size);\
    pthread_setschedprio(&g_thread[g_thread_index], priority);\
    pthread_create(&g_thread[g_thread_index], &g_thread_attr[g_thread_index], fxn, &g_thread_startup[g_thread_index-1]);\
    g_thread_index++;\
}

void *Thread_Startup(void*);
void *Thread_DebugTX(void*);
void *Thread_DebugRx(void*);
void *Thread_RFIntr(void*);

int __errno;
void led_toggle(){
	GPIO_InitTypeDef GPIO_InitStructure;
	volatile int i,j;
	OS_ERR err;
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitStructure.Mode 	= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pin 		= GPIO_PIN_12;
	GPIO_InitStructure.Pull 	= GPIO_PULLUP;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	while(1){
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//		for(i = 0; i < 1000; i++)
//			for(j =0 ; j< 200; j++)
//				asm("NOP");
		 OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_DLY, &err);
	}
}
OS_TCB t_handle;
CPU_STK t_stk[512];
int main(void)
{
    OS_ERR   err;
    int i;

    HAL_Init();                                                 /* See Note 1.                                          */
    Mem_Init();                                                 /* Initialize Memory Managment Module                   */
    Math_Init();                                                /* Initialize Mathematical Module                       */
    BSP_IntDisAll();                                            /* Disable all Interrupts.                              */
    App_OS_SetAllHooks();

    OSInit(&err);                                               /* Init uC/OS-III.                                      */


    for(i = 0; i < APP_THREAD_COUNT-1; i++)
        sem_init(&g_thread_startup[i], 0, 0);
    sem_init(&g_sem_debug, 0, 1);
    g_debug_tx_buffer = mq_open(0, 512);

    pthread_attr_setstacksize(&g_thread_attr[0], 1024);
    pthread_create(&g_thread[0], &g_thread_attr[0], Thread_Startup, 0);
    pthread_setschedprio(&g_thread[0], 1);

    DEFINE_THREAD(Thread_DebugTX, 1024,   1);
    DEFINE_THREAD(Thread_DebugRx, 1024, 1);
    DEFINE_THREAD(Thread_RFIntr,  1024, 3);
    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */

    while (1) {}
}
#include <stdarg.h>
void LREP(char* s, ...){
    uint8_t szBuffer[128];
    int i, len;
    va_list arglist;
    va_start(arglist, s);
    memset(szBuffer, 0, 128);
    _lib_vsnprintf((char*)szBuffer, 127, s, arglist);
    len = strlen_s((char*)szBuffer);
    sem_wait(&g_sem_debug);
    //mq_send(g_debug_tx_buffer, szBuffer, len, 0);
    write(g_fd_debug, szBuffer, len);
    sem_post(&g_sem_debug);
}
void  *Thread_Startup (void *p_arg)
{
	int i;
    struct termios2 opt;
    struct timespec timeout;

    BSP_Init();                                                 /* Initialize BSP functions                             */
    CPU_Init();                                                 /* Initialize the uC/CPU services                       */

    driver_probe();
    board_register_devices();

    g_fd_led[0] = open_dev("led-green", 0);
    g_fd_led[1] = open_dev("led-red", 	0);
    g_fd_led[2] = open_dev("led-blue", 0);
    g_fd_led[3] = open_dev("led-orange", 0);

    g_fd_button = open_dev("button", 	0);
    // open usart
    g_fd_debug = open_dev("usart-1", O_RDWR);
    if(g_fd_debug >= 0){
        // configure
        ioctl(g_fd_debug, TCGETS2, (unsigned int)&opt);
        opt.c_cc[VMIN]  = 1;
        opt.c_cc[VTIME] = 100;
        opt.c_ispeed = 115200;
        opt.c_ospeed = 115200;
        opt.c_cflag &= ~CBAUD;
        opt.c_cflag |= BOTHER;
        /*     no parity
            1 stop bit
            8 bit data
         */
        opt.c_cflag &= ~CSIZE;
        opt.c_cflag |= CS8;
        opt.c_cflag &= ~CSTOPB;
        opt.c_cflag &= ~PARENB;
        opt.c_iflag &= ~INPCK;

        //opt.c_cflag &= ~CRTSCTS;             // disable hardware flow control CTS/ RTS*/
        opt.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls, enable reading
        opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input
        opt.c_oflag &= ~(OPOST|ONLCR|OCRNL);  // raw output
        opt.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK| INLCR| IGNCR| ICRNL); /* disable sofware flow */
        ioctl(g_fd_debug, TCSETS2, (unsigned int)&opt);
        LREP("\r\n____________________________");
        LREP("\r\n|-------- startup ---------|\r\n");
    }else{
    	timeout.tv_sec = 0;
    	timeout.tv_nsec = 1000000*100;
        while(1){
            LED_TOGGLE(RED);
            nanosleep(&timeout, 0);
        };
    }

    if(g_fd_button < 0){
    	LREP("g_fd_button failed\r\n");
    }

    for(i = 0; i < APP_THREAD_COUNT-1; i++)
        sem_post(&g_thread_startup[i]);

    LED_ON(ORANGE);
	timeout.tv_sec = 0;
	timeout.tv_nsec = 1000000*250;
    while (1) {                                          /* Task body, always written as an infinite loop.       */
        LED_TOGGLE(ORANGE);
        nanosleep(&timeout, 0);
    }
}
void *Thread_DebugTX(void* pvParameters){
    uint8_t data[32], len;
    struct timespec timeout;
    sem_t *sem_startup = (sem_t*)pvParameters;

    sem_wait(sem_startup);
    LREP("Thread DebugTx is running.\r\n");

    timeout.tv_sec = 1;
    timeout.tv_nsec = 0;
    while(1){
    	len = mq_timedreceive(g_debug_tx_buffer, data, 32, 0, &timeout);
        if(len > 0)
            write(g_fd_debug, data, len);
    	LED_TOGGLE(BLUE);
    }
}
void *Thread_DebugRx(void* pvParameters){
    int8_t u8val;
    int8_t userinput[33];
    int len, userinput_index = 0;
    sem_t* sem_startup = (sem_t*)pvParameters;
    fd_set readfs;
    struct timeval timeout;

    FD_CLR(g_fd_debug, &readfs);
    timeout.tv_sec  = 0;
    timeout.tv_usec = 1000*100;    // 500ms

    sem_wait(sem_startup);
    LREP("Thread DebugRx is running\r\n");
    while (1) {
        len = select(g_fd_debug, &readfs, 0, 0, &timeout);
        if(len > 0){
            if(FD_ISSET(g_fd_debug, &readfs)){
                len = read_dev(g_fd_debug, &u8val, 1);
                if(len > 0){
                    LREP("%c", u8val);
                    switch(u8val){
						case 's':
							LREP("\r\nsend\r\n");
							break;
						case 'l':
							LREP("\r\nloop\r\n");
							break;
                    }
                    g_debug_cmd = u8val;
                    if(u8val == 'r') reboot();
                }
            }else{
            	LREP("fd %04X %04X\r\n", g_fd_debug, readfs);
            }
        }else if(len == 0){
        }
        else{
            LREP("select uart failed %d.\r\n", len);
            break;
        }
    }
    while(1){sleep(1);}
}
void *Thread_RFIntr(void *pvParameters){
    int8_t buffer[16];
    int len, i;
    sem_t* sem_startup = (sem_t*)pvParameters;
    fd_set readfs;
    struct timeval timeout;
    struct timespec s_timeout = {
    		.tv_sec = 0,
			.tv_nsec = 1000000*500,
    };
    uint8_t led_state = 0;

    sem_wait(sem_startup);
    LREP("Thread RFIntr is running\r\n");

    FD_CLR(g_fd_button, &readfs);
    timeout.tv_sec     = 0;
    timeout.tv_usec = 1000*500;    // 500ms

    while (1) {
        len = select(g_fd_button, &readfs, 0, 0, &timeout);
        if(len > 0){
            if(FD_ISSET(g_fd_button, &readfs)){
            	LREP("Action\r\n");
            }
        }else if(len == 0){
        	LED_TOGGLE(GREEN);
        }else{
            LREP("select intr pin failed.\r\n");
            break;
        }
//    	read_dev(g_fd_button, buffer, 1);
//    	LREP("%d", buffer[0]);
//    	nanosleep(&s_timeout, 0);
    }
    while(1){sleep(1);}
}
// end of file
