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

#include <pthread.h>
#include <unistd.h>

#define BLOCK_

void *ToggleLED_Timer(void*);
void *DetectButtonPress(void*);
void *ToggleLED_IPC(void*);
void initHW();

xQueueHandle pbq;

USART_InitTypeDef g_USART_InitStructure;
GPIO_InitTypeDef  g_GPIOInitStructure;
NVIC_InitTypeDef  g_NVIC_InitStructure;

pthread_t 		g_thread[3];
pthread_attr_t	g_thread_attr[3];
extern int test();
int main(void)
{
  test();
  initHW();
  
  /// gpio
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  g_GPIOInitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  g_GPIOInitStructure.GPIO_Mode = GPIO_Mode_AF;
  g_GPIOInitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  g_GPIOInitStructure.GPIO_OType = GPIO_OType_PP;
  g_GPIOInitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &g_GPIOInitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
  /// uart
  // enable clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  /**uart1 configured as follow
   * baudrate 9600 baud
   * word length 8 bits
   * 1 stop bit
   * no parity
   * hardware flow control disabled
   * receive and transmit enable
   */
  g_USART_InitStructure.USART_BaudRate	= 9600;
  g_USART_InitStructure.USART_WordLength	= USART_WordLength_8b;
  g_USART_InitStructure.USART_StopBits = USART_StopBits_1;
  g_USART_InitStructure.USART_Parity = USART_Parity_No;
  g_USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  g_USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &g_USART_InitStructure);
  USART_Cmd(USART1, ENABLE);
  /// NVIC
//  g_NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//  g_NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  g_NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  g_NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&g_NVIC_InitStructure);
//  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  /* Create IPC variables */
  pbq = xQueueCreate(10, sizeof(int));
  if (pbq == 0) {
    while(1); /* fatal error */
  }
  
  pthread_attr_setstacksize(&g_thread_attr[0], configMINIMAL_STACK_SIZE);
  pthread_setschedprio(&g_thread[0], tskIDLE_PRIORITY + 2UL);
  pthread_create(&g_thread[0], &g_thread_attr[0], ToggleLED_Timer, 0);
  
  pthread_attr_setstacksize(&g_thread_attr[1], configMINIMAL_STACK_SIZE);
  pthread_setschedprio(&g_thread[1], tskIDLE_PRIORITY + 2UL);
  pthread_create(&g_thread[1], &g_thread_attr[1], DetectButtonPress, 0);
  
  pthread_attr_setstacksize(&g_thread_attr[2], configMINIMAL_STACK_SIZE);
  pthread_setschedprio(&g_thread[2], tskIDLE_PRIORITY + 2UL);
  pthread_create(&g_thread[2], &g_thread_attr[2], ToggleLED_IPC, 0);

  /* Start the RTOS Scheduler */
  vTaskStartScheduler();
  
  /* HALT */
  while(1); 
}
void USART_puts(volatile char* s){
	while(*s){
		while(!(USART1->SR & 0x00000040));
		USART_SendData(USART1, *s);
		s++;
	}
}
/**
 * TASK 1: Toggle LED via RTOS Timer
 */
void *ToggleLED_Timer(void *pvParameters){
	USART_puts("\r\n--------- startup ----------\r\n");
  while (1) {
    GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
    
    /*
    Delay for a period of time. vTaskDelay() places the task into
    the Blocked state until the period has expired.
    The delay period is spacified in 'ticks'. We can convert
    yhis in milisecond with the constant portTICK_RATE_MS.
    */
    sleep(1);
    USART_puts("toggle led\r\n");
  }
  return 0;
}

/**
 * TASK 2: Detect Button Press
 * 			And Signal Event via Inter-Process Communication (IPC)
 */
void *DetectButtonPress(void *pvParameters){
  
  int sig = 1;
  
  while (1) {
	/* Detect Button Press  */
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)>0) {
      while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)>0)
        vTaskDelay(100 / portTICK_RATE_MS); /* Button Debounce Delay */
      while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==0)
        vTaskDelay(100 / portTICK_RATE_MS); /* Button Debounce Delay */
      
      xQueueSendToBack(pbq, &sig, 0); /* Send Message */
    }
  }
}

/**
 * TASK 3: Toggle LED via Inter-Process Communication (IPC)
 *
 */
void *ToggleLED_IPC(void *pvParameters) {
  
  int sig;
  portBASE_TYPE status;
  
  while (1) {
    status = xQueueReceive(pbq, &sig, portMAX_DELAY); /* Receive Message */
    												  /* portMAX_DELAY blocks task indefinitely if queue is empty */
    if(status == pdTRUE) {
      GPIO_ToggleBits(GPIOD,GPIO_Pin_14);
    }
  }
}

/**
 * Init HW
 */
void initHW()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure2;
  
  // Init LED
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
    
  // Init PushButton
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure2.GPIO_Pin =  GPIO_Pin_0;
  GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure2);
}
