/*POSIX API*/
#include <drv_api.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <semaphore.h>
#include <mqueue.h>
#include <spidev.h>
#include <drv_gpio.h>

#include <sys/reboot.h>

#include <debug.h>

#include "network/mac/mac_mrf24j40.h"

void *Thread_Startup(void*);
void *Thread_DebugTX(void*);
void *Thread_DebugRx(void *);
void *Thread_RFIntr(void*);
void *Thread_MiwiTask(void*);
void HW_Initalize();
void LREP(char* s, ...);

#define             APP_THREAD_COUNT    5
pthread_t           g_thread[APP_THREAD_COUNT];
pthread_attr_t      g_thread_attr[APP_THREAD_COUNT];
sem_t               g_thread_startup[APP_THREAD_COUNT-1];
sem_t				g_sem_debug;
mqd_t               g_debug_tx_buffer   = 0;
int                 g_fd_debug          = -1;
int                 g_fd_led[4]         = {-1};
int                 g_fd_button         = -1;
volatile uint8_t	g_debug_cmd = 0;

struct mac_mrf24j40	g_rf_mac;
int RF_MAC_callback(void* mac, int type, void* args);

/*************************************************************************/
// AdditionalNodeID variable array defines the additional 
// information to identify a device on a PAN. This array
// will be transmitted when initiate the connection between 
// the two devices. This  variable array will be stored in 
// the Connection Entry structure of the partner device. The 
// size of this array is ADDITIONAL_NODE_ID_SIZE, defined in 
// ConfigApp.h.
// In this demo, this variable array is set to be empty.
/*************************************************************************/
#if ADDITIONAL_NODE_ID_SIZE > 0
    uint8_t AdditionalNodeID[ADDITIONAL_NODE_ID_SIZE] = {0x00};
#endif


extern int board_register_devices();
int g_thread_index = 1;
#define DEFINE_THREAD(fxn, stack_size, priority) {\
    pthread_attr_setstacksize(&g_thread_attr[g_thread_index], stack_size);\
    pthread_setschedprio(&g_thread[g_thread_index], priority);\
    pthread_create(&g_thread[g_thread_index], &g_thread_attr[g_thread_index], fxn, &g_thread_startup[g_thread_index-1]);\
    g_thread_index++;\
}
#define LED_ON(led) {uint8_t val = 1; write(g_fd_led[LED_##led], &val, 1);}
#define LED_OFF(led) {uint8_t val = 0; write(g_fd_led[LED_##led], &val, 1);}
#define LED_TOGGLE(led) {ioctl(g_fd_led[LED_##led], GPIO_IOCTL_TOGGLE, 0);}

/* MIWI
/*------*/
int main(void)
{
    int i;    
    
    HW_Initalize();    
    g_debug_tx_buffer = mq_open(0, 512);

    for(i = 0; i < APP_THREAD_COUNT-1; i++)
        sem_init(&g_thread_startup[i], 0, 0);
    sem_init(&g_sem_debug, 0, 1);
//    sem_init(&g_mimac_access, 0, 1);
    pthread_attr_setstacksize(&g_thread_attr[0], configMINIMAL_STACK_SIZE*32);
    pthread_setschedprio(&g_thread[0], tskIDLE_PRIORITY + 2UL);
    pthread_create(&g_thread[0], &g_thread_attr[0], Thread_Startup, 0);
    
    DEFINE_THREAD(Thread_DebugTX, configMINIMAL_STACK_SIZE*8,  tskIDLE_PRIORITY + 1UL);
    DEFINE_THREAD(Thread_DebugRx, configMINIMAL_STACK_SIZE*32, tskIDLE_PRIORITY + 1UL);
    DEFINE_THREAD(Thread_RFIntr,  configMINIMAL_STACK_SIZE*16, tskIDLE_PRIORITY + 3UL);
    DEFINE_THREAD(Thread_MiwiTask,configMINIMAL_STACK_SIZE*32, tskIDLE_PRIORITY + 4UL);
    
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
    sem_wait(&g_sem_debug);
    mq_send(g_debug_tx_buffer, szBuffer, len, 0);
    sem_post(&g_sem_debug);
}
#define TEST_LEN (110)
struct mac_mrf24j40_open_param	rf_mac_init;
flag_event_t g_test_event;
uint8_t g_rxBuffer[TEST_LEN];
void *Thread_Startup(void *pvParameters){
    int i, ret;
    struct termios2 opt;
    unsigned int uival;
    int myChannel = 11;
    uint8_t pktCMD;
    uint8_t Status;
    uint8_t activeScanIndex=0;
	uint8_t scanresult, RSSIValue;
	struct mac_mrf24j40_write_param rf_trans;
	uint8_t payload[TEST_LEN];
	struct timespec timeout;
	unsigned int timediff, timenow, timeref;
	unsigned int datacnt, speed;
    
    // register drivers & devices
    driver_probe();
    board_register_devices();
    // open gpio
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
        while(1){
            LED_TOGGLE(RED);
            usleep_s(1000 * 100);
        };
    }
    
    rf_mac_init.fd_spi = open_dev("spi-1", O_RDWR);
    if(rf_mac_init.fd_spi < 0){
        LREP("open spi device failed\r\n");
    }
    else{
        uival = SPI_MODE_0;
        if(ioctl(rf_mac_init.fd_spi, SPI_IOC_WR_MODE, (unsigned int)&uival) != 0) LREP("ioctl spi mode failed\r\n");
        uival = 5000000;
        if(ioctl(rf_mac_init.fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, (unsigned int)&uival) != 0) LREP("ioctl spi speed failed\r\n");
        else{
            uival = 0;
            if(ioctl(rf_mac_init.fd_spi, SPI_IOC_RD_MAX_SPEED_HZ, (unsigned int)&uival) == 0) LREP("ioctl spi speed = %u\r\n", uival);
        }
    }
    rf_mac_init.fd_cs = open_dev("spi-1-cs", 0);
    if(rf_mac_init.fd_cs < 0) LREP("open spi cs device failed\r\n");
    rf_mac_init.fd_reset = open_dev("rf-reset", 0);
    if(rf_mac_init.fd_reset < 0) LREP("open rf-reset device failed\r\n");
    rf_mac_init.fd_intr = open_dev("rf-intr", 0);
    if(rf_mac_init.fd_intr < 0) LREP("open rf-intr device failed\r\n");

    App_Initialize();
    rf_trans.altDestAddr = 1;
    rf_trans.altSrcAddr = 1;
    rf_trans.destAddress = 0xFFFF;
    rf_trans.destPANId = 0xFFFF;
    rf_trans.flags.bits.ackReq = 0;
    rf_trans.flags.bits.broadcast = 1;
    rf_trans.flags.bits.destPrsnt = 1;
    rf_trans.flags.bits.sourcePrsnt = 1;
    rf_trans.flags.bits.packetType = MAC_MRF24J40_PACKET_TYPE_DATA;
    rf_trans.flags.bits.repeat = 0;
    rf_trans.flags.bits.secEn = 0;
    flag_event_init(&g_test_event);
    timeout.tv_sec = 1;
    timeout.tv_nsec = 0;
    // signal all other thread startup
    LREP("Thread startup is running\r\n");
    for(i = 0; i < APP_THREAD_COUNT-1; i++){
        sem_post(&g_thread_startup[i]);
    }
    datacnt = 0;
    timediff = 0;
    uint8_t cnt = 0;
    while (1) {
    	if(g_debug_cmd == 's'){
    		for(i =0 ; i < TEST_LEN; i++){
    			payload[i] = cnt++;
    		}
    		MAC_mrf24j40_write(&g_rf_mac, &rf_trans, payload, TEST_LEN);
    		if(flag_event_timedwait(&g_test_event, &timeout) == 0){
    			LREP("timeout, end tx\r\n");
    			PHY_mrf24j40_initialize(&g_rf_mac.phy);
    		    uival = 25;
    		    MAC_mrf24j40_ioctl(&g_rf_mac, mac_mrf24j40_ioc_set_channel, (unsigned int)&uival);
//    			g_debug_cmd = 0;
    		}
    		datacnt += TEST_LEN;

    		for(i =0 ; i < TEST_LEN; i++){
    			payload[i]++;
    			if(g_rxBuffer[i] != payload[i]) break;
    		}
    		if(i < TEST_LEN) {
    			LED_TOGGLE(RED);
    			LREP("failed\r\n");
    		}
    		else {
    			LED_TOGGLE(BLUE);
    		}

    	}else if(g_debug_cmd == 'l'){
    		if(flag_event_timedwait(&g_test_event, &timeout) != 0){
    			for(i =0 ; i < TEST_LEN; i++){
    				g_rxBuffer[i]++;
    			}
    			MAC_mrf24j40_write(&g_rf_mac, &rf_trans, g_rxBuffer, TEST_LEN);
    			LED_TOGGLE(GREEN);
			}else{
				LED_TOGGLE(RED);				
			}
    	}
    	else{
    		if(datacnt > 0){
				timenow = xTaskGetTickCount();
				timediff = (timenow > timeref) ? timenow - timeref : 0;
				if(timediff > 0){
					speed = datacnt*1000/timediff;
					LREP("speed %u bytes in %u ms, %u.%u KB/s\r\n",
							datacnt, timediff,
							speed/1024,
							(speed % 1024) * 10 / 1024);
				}
				datacnt = 0;
    		}
    		usleep_s(100);
    		timeref = xTaskGetTickCount();
    	}
    }
    return 0;
}
void *Thread_DebugTX(void* pvParameters){
    uint8_t data;    
    struct timespec abs_timeout;
    abs_timeout.tv_sec = 1;
    abs_timeout.tv_nsec = 0;
    while(1){
        if(mq_timedreceive(g_debug_tx_buffer, &data, 1, 0, &abs_timeout) == 1)
            write(g_fd_debug, &data, 1);
        //else
        	//portYIELD();
    }
}
void *Thread_RFIntr(void *pvParameters){
    int8_t buffer[16];
    int len, i;
    sem_t* sem_startup = (sem_t*)pvParameters;
    fd_set readfs;
    struct timeval timeout;
    uint8_t led_state = 0;
    
    sem_wait(sem_startup);
    LREP("Thread RFIntr is running\r\n");
    
    FD_CLR(rf_mac_init.fd_intr, &readfs);
    timeout.tv_sec     = 0;
    timeout.tv_usec = 1000*500;    // 500ms

    while (1) {        
        len = select(rf_mac_init.fd_intr, (unsigned int*)&readfs, 0, 0, &timeout);
        if(len > 0){
            if(FD_ISSET(rf_mac_init.fd_intr, &readfs)){
            	MAC_mrf24j40_ioctl(&g_rf_mac, mac_mrf24j40_ioc_trigger_interrupt, 0);
            }
        }else if(len == 0){
        	portYIELD();
        }else{
            LREP("select intr pin failed.\r\n");
            break;
        }
    }
    while(1){sleep(1);}
}
void *Thread_DebugRx(void *pvParameters){
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
        len = select(g_fd_debug, (unsigned int*)&readfs, 0, 0, &timeout);
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
            }
        }else if(len == 0){
        	//portYIELD();
        }
        else{
            LREP("select uart failed %d.\r\n", len);
            break;
        }
    }
    while(1){sleep(1);}
}
void *Thread_MiwiTask(void* pvParameters){
    sem_t* sem_startup = (sem_t*)pvParameters;
    unsigned int uival;


    sem_wait(sem_startup);
    LREP("Thread MiwiTask is running\r\n");
    MAC_mrf24j40_open(&g_rf_mac, &rf_mac_init);
    MAC_mrf24j40_register_callback(&g_rf_mac, RF_MAC_callback, &g_rf_mac);
    uival = 25;
    MAC_mrf24j40_ioctl(&g_rf_mac, mac_mrf24j40_ioc_set_channel, (unsigned int)&uival);
    while(1){
    	if(MAC_mrf24j40_select(&g_rf_mac, 100)){
    		MAC_mrf24j40_task(&g_rf_mac);
    	}
    }
    return 0;
}
int RF_MAC_callback(void* mac, int type, void* args){
	int i;
	struct mac_callback_received_data_args *recvArgs;
	uint8_t* puiVal;
	if(type == mac_callback_type_tx_done){
//		LREP("\r\n[tx:done]\r\n");
//		if(g_debug_cmd == 's')
//			flag_event_post(&g_test_event);
	}
	else if(type == mac_callback_type_tx_false){
		LREP("\r\n[tx:false]\r\n");
	}
	else if(type == mac_callback_type_received_data){
		recvArgs = (struct mac_callback_received_data_args *)args;
#if MAC_PRINT_RX > 0
		LREP("\r\n*** RX BEGIN ***\r\n");
		LREP("len=%d\r\n", recvArgs->packetLen);
		puiVal = (uint8_t*)recvArgs->packet;
		for(i =0 ;i < recvArgs->packetLen; i++){
			if((i % 16) == 0) LREP("\r\n");
			LREP("%02X ", puiVal[i]);
		}
		LREP("\r\nRX END\r\n");
#else
//		if(g_debug_cmd == 'l'){
			i = 0;
			puiVal = (uint8_t*)(recvArgs->packet);
			while(i < recvArgs->packetLen && i < TEST_LEN){
				g_rxBuffer[i] = puiVal[i + 10];
				i++;
			}
			flag_event_post(&g_test_event);
//		}
//		LREP("\r\n[rx]\r\n");
#endif
	}
	return 0;
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


