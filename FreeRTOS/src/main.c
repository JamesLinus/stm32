/* Board includes */
#include "stm32f4_discovery.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include <drv_api.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>

void *Thread_Startup(void*);
void *Thread_2(void*);
void initHW();
void LREP(char* s, ...);

GPIO_InitTypeDef  g_GPIOInitStructure;

#define 			APP_THREAD_COUNT	2
pthread_t 			g_thread[APP_THREAD_COUNT];
pthread_attr_t		g_thread_attr[APP_THREAD_COUNT];
SemaphoreHandle_t 	g_thread_startup[APP_THREAD_COUNT-1];
int					g_fd_uart1 = -1;

extern int board_register_devices();
void error_trap(){
	int i = 0;
	while(1){
		GPIO_ToggleBits(GPIOD,GPIO_Pin_14);
		for(i = 0; i < 10000; i++){
		}
	}
}
int main(void)
{
	int i;

	initHW();

	for(i = 0; i < APP_THREAD_COUNT-1; i++)
		g_thread_startup[i] = xSemaphoreCreateBinary();

	pthread_attr_setstacksize(&g_thread_attr[0], configMINIMAL_STACK_SIZE);
	pthread_setschedprio(&g_thread[0], tskIDLE_PRIORITY + 3UL);
	pthread_create(&g_thread[0], &g_thread_attr[0], Thread_Startup, 0);

	pthread_attr_setstacksize(&g_thread_attr[1], configMINIMAL_STACK_SIZE);
	pthread_setschedprio(&g_thread[1], tskIDLE_PRIORITY + 2UL);
	pthread_create(&g_thread[1], &g_thread_attr[1], Thread_2, &g_thread_startup[0]);

	/* Start the RTOS Scheduler */
	vTaskStartScheduler();

	/* HALT */
	while(1);
}
#include <stdarg.h>
void LREP(char* s, ...){
    char szBuffer[128];
    va_list arglist;
    va_start(arglist, s);
    memset(szBuffer, 0, 128);
    vsnprintf(szBuffer, 127, s, arglist);
    write(g_fd_uart1, szBuffer, strlen(szBuffer));
}
/**
 * TASK 1: Toggle LED via RTOS Timer
 */
void *Thread_Startup(void *pvParameters){
	int i;
	// register drivers & devices
	driver_probe();
	board_register_devices();
	// open device
	g_fd_uart1 = open_dev("usart-1", O_RDWR);
	if(g_fd_uart1 >= 0){
		LREP("\r\n--------- startup ----------\r\n");
	}else{
		error_trap();
	}
	// signal all other thread
	LREP("Thread startup is running\r\n");
	for(i = 0; i < 1; i++){
		xSemaphoreGive(g_thread_startup[i]);
	}
	while (1) {
		GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
		sleep(1);
		LREP(".");
	}
	return 0;
}

/**
 * TASK 2: Detect Button Press
 * 			And Signal Event via Inter-Process Communication (IPC)
 */
void *Thread_2(void *pvParameters){
	int8_t buffer[16];
	int len, i;
	SemaphoreHandle_t* sem_startup = (SemaphoreHandle_t*)pvParameters;

	xSemaphoreTake(*sem_startup, portMAX_DELAY);
	LREP("Thread 2 is running\r\n");
	while (1) {
	  len = read_dev(g_fd_uart1, buffer, 1);
//	  LREP("read %d\r\n", len);
	  for(i = 0; i < len; i++){
		  LREP("%c", buffer[i]);
	  }
	  GPIO_ToggleBits(GPIOD,GPIO_Pin_14);
	}
}
/**
 * Init HW
 */
void initHW()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  // Init LED
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
