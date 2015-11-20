#include <drv_api.h>
#include <drv_usart.h>
#include <drv_gpio.h>

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <termios.h>

int 	usart_init		(void);
int 	usart_open		(struct platform_device *dev, int flags);
int 	usart_close		(struct platform_device *dev);
int		usart_write		(struct platform_device *dev, const void* buf, int count);
int		usart_read		(struct platform_device *dev, void* buf, int count);
int		usart_ioctl		(struct platform_device *dev, int request, unsigned int arguments);
int		usart_select	(struct platform_device *device, int *readfd, int *writefd, int *exceptfd, int timeout);

struct usart_driver_arch_data{
	void* rx_event[USART_MODULE_COUNT];
};
struct usart_driver_arch_data g_usart_driver_arch_data;

static struct platform_driver g_usart_driver = {
	.driver		= {
		.name	= "usart-drv",
		.devices = 0,
	},
	.archdata = &g_usart_driver_arch_data,
	.probe		= 0,
	.remove		= 0,
	.resume 	= 0,
	.suspend	= 0,
	.shutdown	= 0,

	.open 		= &usart_open,
	.close 		= &usart_close,
	.read 		= &usart_read,
	.write 		= &usart_write,
	.ioctl 		= &usart_ioctl,
	.select		= &usart_select,

	.next 		= 0,
};

int usart_init		(void){;
	memset(&g_usart_driver_arch_data, 0, sizeof(g_usart_driver_arch_data));
	platform_driver_register(&g_usart_driver);
	return 0;
}
module_init(usart_init);
int 	usart_open		(struct platform_device *dev, int flags){
	int ret = -EPERM;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;
	uint32_t RCC_AHB1Periph;
	uint16_t GPIO_PinSource;
	uint8_t GPIO_AF;
	uint8_t NVIC_IRQChannel;
	unsigned int bank, pin;	
	GPIO_TypeDef* GPIOx;
	USART_TypeDef* USARTx;
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef  GPIOInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	if(!dev || dev->id < 0 || dev->id >= USART_MODULE_COUNT) return ret;
	if(dev->id == 0){
		// usart1
		GPIO_AF 		= GPIO_AF_USART1;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		USARTx = USART1;
		NVIC_IRQChannel = USART1_IRQn;
	}else if(dev->id == 1){
		// usart2
		GPIO_AF 		= GPIO_AF_USART2;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		USARTx = USART2;
		NVIC_IRQChannel = USART2_IRQn;
	}
	else if(dev->id == 2){
		// usart3
		GPIO_AF 		= GPIO_AF_USART3;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		USARTx = USART3;
		NVIC_IRQChannel = USART3_IRQn;
	}
	else if(dev->id == 3){
		// usart4
		GPIO_AF 		= GPIO_AF_UART4;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		USARTx = UART4;
		NVIC_IRQChannel = UART4_IRQn;
	}
	else if(dev->id == 4){
		//PC12
		GPIO_AF 		= GPIO_AF_UART5;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
		USARTx = UART5;
		NVIC_IRQChannel = UART5_IRQn;
	}
	else if(dev->id == 5){
		// usart6 PG14 PG9
		GPIO_AF 		= GPIO_AF_USART6;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
		USARTx = USART6;
		NVIC_IRQChannel = USART6_IRQn;
	}
	// config gpio
	GPIOInitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIOInitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIOInitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIOInitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	// rx
	bank = gpio_get_bank_index(data->rx_pin);
	pin  = gpio_get_pin_index(data->rx_pin);
	RCC_AHB1PeriphClockCmd(g_gpio_bank_ref[bank].RCC_AHB1Periph_GPIOx, ENABLE);
	GPIOInitStructure.GPIO_Pin 	= g_gpio_pin_ref[pin].GPIO_Pin;
	GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIOInitStructure);
	GPIO_PinAFConfig(g_gpio_bank_ref[bank].GPIOx, g_gpio_pin_ref[pin].GPIO_PinSource, GPIO_AF);
	
	// tx
	bank = gpio_get_bank_index(data->tx_pin);
	pin  = gpio_get_pin_index(data->tx_pin);
	RCC_AHB1PeriphClockCmd(g_gpio_bank_ref[bank].RCC_AHB1Periph_GPIOx, ENABLE);
	GPIOInitStructure.GPIO_Pin 	= g_gpio_pin_ref[pin].GPIO_Pin;
	GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIOInitStructure);
	GPIO_PinAFConfig(g_gpio_bank_ref[bank].GPIOx, g_gpio_pin_ref[pin].GPIO_PinSource, GPIO_AF);
	// config usart	
	/**uart1 configured as follow
	* baudrate 9600 baud
	* word length 8 bits
	* 1 stop bit
	* no parity
	* hardware flow control disabled
	* receive and transmit enable
	*/
	USART_InitStructure.USART_BaudRate		= 9600;
	USART_InitStructure.USART_WordLength	= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits 		= USART_StopBits_1;
	USART_InitStructure.USART_Parity 		= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode 			= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USARTx, &USART_InitStructure);
	// interrupt
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USARTx, ENABLE);
	data->__drv_usart_base = (void*)USARTx;
	if(!g_usart_driver_arch_data.rx_event[dev->id])
		g_usart_driver_arch_data.rx_event[dev->id] = xQueueCreate(32, 1);
	ret = 0;
	return ret;
}
int 	usart_close		(struct platform_device *dev){
	int ret = -EPERM;
	USART_TypeDef* USARTx;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;

	if(!dev || dev->id < 0 || dev->id >= USART_MODULE_COUNT) return ret;
	USART_Cmd((USART_TypeDef*)data->__drv_usart_base, DISABLE);
	ret = 0;
	return ret;
}
extern void LREP(char* s, ...);
int		usart_read		(struct platform_device *dev, void* buf, int count){
	int ret = -EPERM;
	USART_TypeDef* USARTx;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;
	unsigned char* p = (unsigned char*)buf;

	if(!dev || dev->id < 0 || dev->id >= USART_MODULE_COUNT) return ret;
	ret = 0;
	while(count > 0){
		if(xQueueReceive(g_usart_driver_arch_data.rx_event[dev->id], p, 0) != pdTRUE)
			break;
		count --;
		p++;
		ret++;
	}
	return ret;
}
int		usart_write	(struct platform_device *dev, const void* buf, int count){
	int ret = -EPERM;
	USART_TypeDef* USARTx;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;
	unsigned char* p = (unsigned char*)buf;
	if(!dev || dev->id < 0 || dev->id >= USART_MODULE_COUNT) return ret;

	ret = 0;
	while(count > 0){
		while(!(((USART_TypeDef*)data->__drv_usart_base)->SR & USART_SR_TC)){}
		USART_SendData((USART_TypeDef*)data->__drv_usart_base, *p);
		count --;
		p++;
		ret++;
	}
	return ret;
}
int		usart_ioctl	(struct platform_device *dev, int request, unsigned int arguments){
	int ret = -EPERM;
	struct termios2* opt = 0;
	USART_InitTypeDef USART_InitStructure;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;
	
	switch(request){
		case TCSETS2:{
			opt = (struct termios2*)arguments;
			USART_InitStructure.USART_BaudRate		= opt->c_ispeed;
			USART_InitStructure.USART_WordLength	= USART_WordLength_8b;
			USART_InitStructure.USART_StopBits 		= USART_StopBits_1;
			USART_InitStructure.USART_Parity 		= USART_Parity_No;
			USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode 			= USART_Mode_Rx | USART_Mode_Tx;
			USART_Init((USART_TypeDef*)data->__drv_usart_base, &USART_InitStructure);
			ret = 0;
			break;
		}
		default:
			break;
	}
	
	return ret;
}
int		usart_select(struct platform_device *dev, int *readfd, int *writefd, int *exceptfd, int timeout){
	int ret = -EPERM;
	uint8_t u8data;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;
	if(readfd) 		*readfd = 0;
	if(writefd) 	*writefd = 0;
	if(exceptfd) 	*exceptfd = 0;
	if(readfd){
		ret = xQueuePeek(g_usart_driver_arch_data.rx_event[dev->id], &u8data, timeout);
		if(ret == pdTRUE) {
			*readfd = 1;
			ret = 1;
		}
		else ret = 0;
	}	
	return ret;
}
// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART1_IRQHandler(void){
	static uint8_t data;
	static BaseType_t xHigherPriorityTaskWoken;
	static USART_TypeDef* USARTx = USART1;
	
	if( USART_GetITStatus(USARTx, USART_IT_RXNE) ){
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		data = USART_ReceiveData(USARTx);
		xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(g_usart_driver_arch_data.rx_event[0], &data, &xHigherPriorityTaskWoken);
	}
}
void USART2_IRQHandler(void){
	static uint8_t data;
	static BaseType_t xHigherPriorityTaskWoken;
	static USART_TypeDef* USARTx = USART2;
	
	if( USART_GetITStatus(USARTx, USART_IT_RXNE) ){
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		data = USART_ReceiveData(USARTx);
		xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(g_usart_driver_arch_data.rx_event[1], &data, &xHigherPriorityTaskWoken);
	}
}
void USART3_IRQHandler(void){
	static uint8_t data;
	static BaseType_t xHigherPriorityTaskWoken;
	static USART_TypeDef* USARTx = USART3;
	
	if( USART_GetITStatus(USARTx, USART_IT_RXNE) ){
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		data = USART_ReceiveData(USARTx);
		xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(g_usart_driver_arch_data.rx_event[2], &data, &xHigherPriorityTaskWoken);
	}
}
void UART4_IRQHandler(void){
	static uint8_t data;
	static BaseType_t xHigherPriorityTaskWoken;
	static USART_TypeDef* USARTx = UART4;
	
	if( USART_GetITStatus(USARTx, USART_IT_RXNE) ){
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		data = USART_ReceiveData(USARTx);
		xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(g_usart_driver_arch_data.rx_event[3], &data, &xHigherPriorityTaskWoken);
	}
}
void UART5_IRQHandler(void){
	static uint8_t data;
	static BaseType_t xHigherPriorityTaskWoken;
	static USART_TypeDef* USARTx = UART5;
	
	if( USART_GetITStatus(USARTx, USART_IT_RXNE) ){
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		data = USART_ReceiveData(USARTx);
		xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(g_usart_driver_arch_data.rx_event[4], &data, &xHigherPriorityTaskWoken);
	}
}
void USART6_IRQHandler(void){
	static uint8_t data;
	static BaseType_t xHigherPriorityTaskWoken;
	static USART_TypeDef* USARTx = USART6;
	
	if( USART_GetITStatus(USARTx, USART_IT_RXNE) ){
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		data = USART_ReceiveData(USARTx);
		xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(g_usart_driver_arch_data.rx_event[5], &data, &xHigherPriorityTaskWoken);
	}
}
