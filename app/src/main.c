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
#include <rf_port.h>

#include "system.h"
#include "system_config.h"
#include "miwi/miwi_api.h"

void *Thread_Startup(void*);
void *Thread_DebugTX(void*);
void *Thread_DebugRx(void *);
void *Thread_RFIntr(void*);
void HW_Initalize();
void LREP(char* s, ...);

#define             APP_THREAD_COUNT    4
pthread_t           g_thread[APP_THREAD_COUNT];
pthread_attr_t      g_thread_attr[APP_THREAD_COUNT];
sem_t               g_thread_startup[APP_THREAD_COUNT-1];
sem_t				g_sem_debug;
mqd_t               g_debug_tx_buffer   = 0;
int                 g_fd_debug          = -1;
int                 g_fd_led[4]         = {-1};
int                 g_fd_button         = -1;
struct mrf24j40_device	g_rf_dev;
volatile uint8_t	g_debug_cmd = 0;

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
void LED_ON(int index){
	uint8_t val = 0;
	if(index >= 0 && index < 4){
		write(g_fd_led[index], &val, 1);
	}
}
void LED_OFF(int index){
	uint8_t val = 1;
	if(index >= 0 && index < 4){
		write(g_fd_led[index], &val, 1);
	}
}
void LED_TOGGLE(int index){
	if(index >= 0 && index < 4){
		ioctl(g_fd_led[index], GPIO_IOCTL_TOGGLE, 0);
	}
}
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
    pthread_attr_setstacksize(&g_thread_attr[0], configMINIMAL_STACK_SIZE*20);
    pthread_setschedprio(&g_thread[0], tskIDLE_PRIORITY + 2UL);
    pthread_create(&g_thread[0], &g_thread_attr[0], Thread_Startup, 0);
    
    DEFINE_THREAD(Thread_DebugTX,     configMINIMAL_STACK_SIZE*20, tskIDLE_PRIORITY + 1UL);
    DEFINE_THREAD(Thread_DebugRx,     configMINIMAL_STACK_SIZE*20, tskIDLE_PRIORITY + 1UL);
    DEFINE_THREAD(Thread_RFIntr,     configMINIMAL_STACK_SIZE*20,  tskIDLE_PRIORITY + 3UL);
    
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
void *Thread_Startup(void *pvParameters){
    int i, ret;
    struct termios2 opt;
    unsigned int uival;
    int myChannel = 11;
    uint8_t pktCMD;
    bool bval;
    
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
            LED_TOGGLE(LED_RED);
            usleep_s(1000 * 100);
        };
    }
    
    g_rf_dev.fd_spi = open_dev("spi-1", O_RDWR);
    if(g_rf_dev.fd_spi < 0){
        LREP("open spi device failed\r\n");
    }
    else{
        uival = SPI_MODE_0;
        if(ioctl(g_rf_dev.fd_spi, SPI_IOC_WR_MODE, (unsigned int)&uival) != 0) LREP("ioctl spi mode failed\r\n");
        uival = 1000000;
        if(ioctl(g_rf_dev.fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, (unsigned int)&uival) != 0) LREP("ioctl spi speed failed\r\n");
        else{
            uival = 0;
            if(ioctl(g_rf_dev.fd_spi, SPI_IOC_RD_MAX_SPEED_HZ, (unsigned int)&uival) == 0) LREP("ioctl spi speed = %u\r\n", uival);
        }
    }
    g_rf_dev.fd_cs = open_dev("spi-1-cs", 0);
    if(g_rf_dev.fd_cs < 0) LREP("open spi cs device failed\r\n");
    g_rf_dev.fd_reset = open_dev("rf-reset", 0);
    if(g_rf_dev.fd_reset < 0) LREP("open rf-reset device failed\r\n");
    g_rf_dev.fd_intr = open_dev("rf-intr", 0);
    if(g_rf_dev.fd_intr < 0) LREP("open rf-intr device failed\r\n");
    
    // signal all other thread startup
    LREP("Thread startup is running\r\n");
    for(i = 0; i < APP_THREAD_COUNT-1; i++){
        sem_post(&g_thread_startup[i]);
    }
    /* MIWI */
	/*******************************************************************/
	// Initialize the MiWi Protocol Stack. The only input parameter indicates
	// if previous network configuration should be restored.
	/*******************************************************************/
    MiApp_ProtocolInit(false);
	/*******************************************************************/
	// Set Device Communication Channel
	/*******************************************************************/
	myChannel = 11;
	if( MiApp_SetChannel(myChannel) == false )
	{
		LREP("ERROR: Unable toSet Channel..\r\n");
	}
	/*******************************************************************/
	//  Set the connection mode. The possible connection modes are:
	//      ENABLE_ALL_CONN:    Enable all kinds of connection
	//      ENABLE_PREV_CONN:   Only allow connection already exists in 
	//                          connection table
	//      ENABL_ACTIVE_SCAN_RSP:  Allow response to Active scan
	//      DISABLE_ALL_CONN:   Disable all connections. 
	/*******************************************************************/
	MiApp_ConnectionMode(ENABLE_ALL_CONN);
	g_debug_cmd = 0;
	LREP("cmd? ");
	while(g_debug_cmd == 0){
		sleep(1);
	}
	if(g_debug_cmd == 'c'){
		// create network
		MiApp_ProtocolInit(false);
		MiApp_StartConnection(START_CONN_DIRECT, 0, 0);
		LREP("Created Network Successfully\r\n");

		LREP("PANID:%02x%02x Ch:%02d\r\n",myPANID.v[1],myPANID.v[0],myChannel);
		LREP("Address: %02x%02x\r\n", myShortAddress.v[1], myShortAddress.v[0]);	
		
		/*******************************************************************/
		// Wait for a Node to Join Network then proceed to Demo's
		/*******************************************************************/
		LREP("Wait node join network...\r\n");
		while(!ConnectionTable[0].status.bits.isValid)
		{
			usleep_s(1000);
			if(MiApp_MessageAvailable())
				MiApp_DiscardMessage();
		}
		LREP("Node join network\r\n");
	}else if(g_debug_cmd == 'j'){
		volatile uint8_t scanresult;
		LREP("  Scanning for    Networks....\r\n");
		LREP("Please Select   Network to Join \r\n");
		MiApp_ProtocolInit(false);

		/*******************************************************************/
		// Perform an active scan
		/*******************************************************************/
		if(myChannel < 8)
		scanresult = MiApp_SearchConnection(10, (0x00000001 << myChannel));
		else if(myChannel < 16)
		scanresult = MiApp_SearchConnection(10, (0x00000100 << (myChannel-8)));
		else if(myChannel < 24)
		scanresult = MiApp_SearchConnection(10, (0x00010000 << (myChannel-16)));
		else
		scanresult = MiApp_SearchConnection(10, (0x01000000 << (myChannel-24)));
		for(i = 0; i < scanresult; i++){
			LREP("<PANID:%02x%02x>\r\n",ActiveScanResults[i].PANID.v[1], ActiveScanResults[i].PANID.v[0]);
		}
		if(scanresult == 0) LREP("No network found\r\n");
	}
	//
    while (1) {
    	bval = MiApp_MessageAvailable();
        if(bval)
		{
			pktCMD = rxMessage.Payload[0];
			LREP("cmd=%X\r\n", pktCMD);
		}else usleep_s(1000);
    }
    while(1){sleep(1);}
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
    
    FD_CLR(g_rf_dev.fd_intr, &readfs);
    timeout.tv_sec     = 0;
    timeout.tv_usec = 1000*500;    // 500ms

    while (1) {        
        len = select(g_rf_dev.fd_intr, (unsigned int*)&readfs, 0, 0, &timeout);
        if(len > 0){
            if(FD_ISSET(g_rf_dev.fd_intr, &readfs)){
                LREP("i");
                rf_port_set_if();
                drv_mrf_handler();
                rf_port_clear_if();
            }
        }else if(len == 0){
        }else{
            LREP("select intr pin failed.\r\n");
            break;
        }
    }
    while(1){sleep(1);}
}
void *Thread_DebugRx(void *pvParameters){
    int8_t buffer[16];
    int len;
    sem_t* sem_startup = (sem_t*)pvParameters;
    fd_set readfs;
    struct timeval timeout;

    FD_CLR(g_fd_debug, &readfs);
    timeout.tv_sec  = 0;
    timeout.tv_usec = 1000*500;    // 500ms

    sem_wait(sem_startup);
    LREP("Thread DebugRx is running\r\n");
    while (1) {
        len = select(g_fd_debug, (unsigned int*)&readfs, 0, 0, &timeout);
        if(len > 0){
            if(FD_ISSET(g_fd_debug, &readfs)){
                len = read_dev(g_fd_debug, buffer, 1);
                if(len > 0){
                    LREP("%c", buffer[0]);
                    if(buffer[0] == 'r') reboot();
                    g_debug_cmd = buffer[0];
                }
            }
        }else if(len == 0){
            LED_TOGGLE(LED_GREEN);
        }
        else{
            LREP("select uart failed %d.\r\n", len);
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

