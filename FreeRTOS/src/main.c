/*POSIX API*/
#include <drv_api.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <semaphore.h>
#include <mqueue.h>
#include <spidev.h>

#include <sys/reboot.h>

void *Thread_Startup(void*);
void *Thread_UartTX(void*);
void *Thread_2(void*);
void HW_Initalize();
void LREP(char* s, ...);

#define 			APP_THREAD_COUNT	3
pthread_t 			g_thread[APP_THREAD_COUNT];
pthread_attr_t		g_thread_attr[APP_THREAD_COUNT];
sem_t 				g_thread_startup[APP_THREAD_COUNT-1];
int					g_fd_uart1 			= -1;
int					g_fd_led_red 		= -1;
int					g_fd_led_green 		= -1;
int					g_fd_button			= -1;
int 				g_fd_spi			= -1;
mqd_t				g_uart_tx_buffer 	= 0;

extern int board_register_devices();

int g_thread_index = 1;
#define DEFINE_THREAD(fxn, stack_size, priority) {\
	pthread_attr_setstacksize(&g_thread_attr[g_thread_index], stack_size);\
	pthread_setschedprio(&g_thread[g_thread_index], priority);\
	pthread_create(&g_thread[g_thread_index], &g_thread_attr[g_thread_index], fxn, &g_thread_startup[g_thread_index-1]);\
	g_thread_index++;\
}
int main(void)
{
	int i;	
	
	HW_Initalize();	
	g_uart_tx_buffer = mq_open(0, 128);

	for(i = 0; i < APP_THREAD_COUNT-1; i++)
		sem_init(&g_thread_startup[i], 0, 0);

	pthread_attr_setstacksize(&g_thread_attr[0], configMINIMAL_STACK_SIZE*5);
	pthread_setschedprio(&g_thread[0], tskIDLE_PRIORITY + 2UL);
	pthread_create(&g_thread[0], &g_thread_attr[0], Thread_Startup, 0);
	
	DEFINE_THREAD(Thread_UartTX, 	configMINIMAL_STACK_SIZE, 	tskIDLE_PRIORITY + 2UL);
	DEFINE_THREAD(Thread_2, 		configMINIMAL_STACK_SIZE*2, tskIDLE_PRIORITY + 2UL);
	
	/* Start the RTOS Scheduler */
	vTaskStartScheduler();

	/* HALT */
	while(1);
}
#include <stdarg.h>
void LREP(char* s, ...){
    char szBuffer[128];
    int i, len;
    va_list arglist;
    va_start(arglist, s);
    memset(szBuffer, 0, 128);
    vsnprintf(szBuffer, 127, s, arglist);
    len = strlen(szBuffer);
	mq_send(g_uart_tx_buffer, szBuffer, len, 0);
}
void *Thread_Startup(void *pvParameters){
	int i, ret;
	struct termios2 opt;
	uint8_t u8data, u8data2, led_status_state;
	uint8_t u8button = 0;
	fd_set readfs;
	struct timeval timeout;
	struct spi_ioc_transfer spi_tr;
	int8_t rx_buf[16];
	int8_t tx_buf[16];	
	int spi1_cs = -1;
	
	// register drivers & devices
	driver_probe();
	board_register_devices();
	// open gpio
	g_fd_led_red = open_dev("led-red", 0);
	g_fd_led_green = open_dev("led-green", 0);
	g_fd_button = open_dev("button", 0);
	// open usart
	g_fd_uart1 = open_dev("lrep", O_RDWR);
	if(g_fd_uart1 >= 0){
		// configure
		ioctl(g_fd_uart1, TCGETS2, (unsigned int)&opt);
		opt.c_cc[VMIN]  = 1;
		opt.c_cc[VTIME] = 100;
        opt.c_ispeed = 115200;
        opt.c_ospeed = 115200;
        opt.c_cflag &= ~CBAUD;
        opt.c_cflag |= BOTHER;
		/* 	no parity
			1 stop bit
			8 bit data
		 */
		opt.c_cflag &= ~CSIZE;
		opt.c_cflag |= CS8;
		opt.c_cflag &= ~CSTOPB;
		opt.c_cflag &= ~PARENB;
		opt.c_iflag &= ~INPCK;

		//opt.c_cflag &= ~CRTSCTS; 			// disable hardware flow control CTS/ RTS*/
		opt.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls, enable reading
		opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input
		opt.c_oflag &= ~(OPOST|ONLCR|OCRNL);  // raw output
		opt.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK| INLCR| IGNCR| ICRNL); /* disable sofware flow */
		ioctl(g_fd_uart1, TCSETS2, (unsigned int)&opt);		
		LREP("\r\n____________________________");
		LREP("\r\n|-------- startup ---------|\r\n");
	}else{
		while(1){
			write(g_fd_led_red, &led_status_state, 1);
			led_status_state = !led_status_state;
			usleep_s(1000 * 100);
		};
	}	
	g_fd_spi = open_dev("spi-1", O_RDWR);
	if(g_fd_spi < 0) LREP("open spi device failed\r\n");
	spi1_cs = open_dev("spi-1-cs", 0);
	if(spi1_cs < 0) LREP("open spi cs device failed\r\n");
	u8data = 1;
	write(spi1_cs, &u8data, 1);
	// signal all other thread startup
	LREP("Thread startup is running\r\n");
	for(i = 0; i < APP_THREAD_COUNT-1; i++){
		sem_post(&g_thread_startup[i]);
	}
	
	spi_tr.speed_hz = 0;
	spi_tr.bits_per_word = 0;
	spi_tr.tx_buf = (unsigned int)tx_buf;
	spi_tr.rx_buf = (unsigned int)rx_buf;
	spi_tr.len    = 6;
	tx_buf[0]= 0x9F;
	while (1) {
		u8data = 0;
		memset(rx_buf, 0, 6);
		write(spi1_cs, &u8data, 1);
		ret = ioctl(g_fd_spi, SPI_IOC_MESSAGE(1),(unsigned int) &spi_tr);
		u8data = 1;
		write(spi1_cs, &u8data, 1);
		LREP("spi read %d\r\n", ret);
		if(ret == spi_tr.len){
			for(i=0;i<ret; i++){
				LREP("%02X ", (unsigned char)rx_buf[i]);
			}
			LREP("\r\n");
		}
		sleep(1);
	}
	while(1){sleep(1);}
	return 0;
}
void *Thread_UartTX(void* pvParameters){
	uint8_t data;	
	struct timespec abs_timeout;
	abs_timeout.tv_sec = 1;
	abs_timeout.tv_nsec = 0;
	while(1){
		if(mq_timedreceive(g_uart_tx_buffer, &data, 1, 0, &abs_timeout) == 1)
			write(g_fd_uart1, &data, 1);
	}
}
void *Thread_2(void *pvParameters){
	int8_t buffer[16];
	int len, i;
	sem_t* sem_startup = (sem_t*)pvParameters;
	fd_set readfs;
	struct timeval timeout;
	uint8_t led_state = 0;
	
	FD_CLR(g_fd_uart1, &readfs);
	timeout.tv_sec 	= 0;
	timeout.tv_usec = 1000*500;	// 500ms

	sem_wait(sem_startup);
	LREP("Thread 2 is running\r\n");
	while (1) {		
		len = select(g_fd_uart1, &readfs, 0, 0, &timeout);
		if(len > 0){
			if(FD_ISSET(g_fd_uart1, &readfs)){
				len = read_dev(g_fd_uart1, buffer, 1);
				for(i = 0; i < len;  i++){
				  LREP("%c", buffer[i]);
				  if(buffer[i] == 'r'){
					  LREP("\r\nSW RESET\r\n");
					  sleep(1);
					  reboot();
				  }
				}
				write(g_fd_led_red, &led_state, 1);
				led_state = !led_state;
			}
		}else if(len == 0){
		}else{
			LREP("select failed.\r\n");
			break;
		}
	}
	while(1){sleep(1);}
}
/**
 * Init HW
 */
/* Board includes */
#include "stm32f4_discovery.h"
#include "stm32f4xx_rcc.h"
void HW_Initalize()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}
