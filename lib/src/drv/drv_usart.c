#include "../include/drv_api.h"
#include "../include/drv_usart.h"

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

int 	usart_init		(void);
int 	usart_open		(struct platform_device *dev, int flags);
int 	usart_close		(struct platform_device *dev);
int		usart_write		(struct platform_device *dev, const void* buf, int count);
int		usart_read		(struct platform_device *dev, void* buf, int count);

struct usart_driver_arch_data{
	void* tx_event[USART_MODULE_COUNT];
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
	.ioctl 		= 0,

	.next 		= 0,
};
//extern void USART_puts(volatile char* s);
//extern void USART_putu16(uint16_t val);
//extern void USART_putu32(uint32_t val);

int usart_init		(void){;
	platform_driver_register(&g_usart_driver);
	return 0;
}
module_init(usart_init);
int 	usart_open		(struct platform_device *dev, int flags){
	int ret = -EPERM;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;

	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef  GPIOInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint32_t RCC_AHB1Periph;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_PinSource_Tx, GPIO_PinSource_Rx;
	uint8_t GPIO_AF;
	USART_TypeDef* USARTx;
	uint8_t NVIC_IRQChannel;

	if(!dev || dev->id < 0 || dev->id >= USART_MODULE_COUNT) return ret;
	// config gpio
	GPIOInitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIOInitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIOInitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIOInitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	if(dev->id == 0){
		// usart1
		RCC_AHB1Periph 				= RCC_AHB1Periph_GPIOB;
		GPIOInitStructure.GPIO_Pin 	= GPIO_Pin_6 | GPIO_Pin_7;
		GPIOx 						= GPIOB;
		GPIO_PinSource_Tx 			= GPIO_PinSource6;
		GPIO_PinSource_Rx 			= GPIO_PinSource7;
		GPIO_AF 					= GPIO_AF_USART1;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		USARTx = USART1;
		NVIC_IRQChannel = USART1_IRQn;
	}else if(dev->id == 1){
		// usart2
		RCC_AHB1Periph 				= RCC_AHB1Periph_GPIOA;
		GPIOInitStructure.GPIO_Pin 	= GPIO_Pin_2 | GPIO_Pin_3;
		GPIOx 						= GPIOA;
		GPIO_PinSource_Tx 			= GPIO_PinSource2;
		GPIO_PinSource_Rx 			= GPIO_PinSource3;
		GPIO_AF 					= GPIO_AF_USART2;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		USARTx = USART2;
		NVIC_IRQChannel = USART2_IRQn;
	}
	else if(dev->id == 2){
		// usart3
		RCC_AHB1Periph 				= RCC_AHB1Periph_GPIOD;
		GPIOInitStructure.GPIO_Pin 	= GPIO_Pin_8 | GPIO_Pin_9;
		GPIOx 						= GPIOD;
		GPIO_PinSource_Tx 			= GPIO_PinSource8;
		GPIO_PinSource_Rx 			= GPIO_PinSource9;
		GPIO_AF 					= GPIO_AF_USART3;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		USARTx = USART3;
		NVIC_IRQChannel = USART3_IRQn;
	}
	else if(dev->id == 3){
		// usart4
		RCC_AHB1Periph 				= RCC_AHB1Periph_GPIOC;
		GPIOInitStructure.GPIO_Pin 	= GPIO_Pin_10 | GPIO_Pin_11;
		GPIOx 						= GPIOC;
		GPIO_PinSource_Tx 			= GPIO_PinSource10;
		GPIO_PinSource_Rx 			= GPIO_PinSource11;
		GPIO_AF 					= GPIO_AF_UART4;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		USARTx = UART4;
		NVIC_IRQChannel = UART4_IRQn;
	}
	else if(dev->id == 4){
		//PC12
		RCC_AHB1Periph 				= RCC_AHB1Periph_GPIOC;
		GPIOInitStructure.GPIO_Pin 	= GPIO_Pin_12;
		GPIOx 						= GPIOC;
		GPIO_PinSource_Tx 			= GPIO_PinSource12;
		GPIO_AF 					= GPIO_AF_UART5;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph, ENABLE);
		GPIO_Init(GPIOx, &GPIOInitStructure);
		GPIO_PinAFConfig(GPIOx, GPIO_PinSource_Tx, GPIO_AF);
		GPIO_PinAFConfig(GPIOx, GPIO_PinSource_Rx, GPIO_AF);
		// usart5
		RCC_AHB1Periph 				= RCC_AHB1Periph_GPIOD;
		GPIOInitStructure.GPIO_Pin 	= GPIO_Pin_2;
		GPIOx 						= GPIOD;
		GPIO_PinSource_Tx 			= GPIO_PinSource2;
		GPIO_AF 					= GPIO_AF_UART5;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
		USARTx = UART5;
		NVIC_IRQChannel = UART5_IRQn;
	}
	else if(dev->id == 5){
		// usart6 PG14 PG9
		RCC_AHB1Periph 				= RCC_AHB1Periph_GPIOG;
		GPIOInitStructure.GPIO_Pin 	= GPIO_Pin_9 | GPIO_Pin_14;
		GPIOx 						= GPIOG;
		GPIO_PinSource_Tx 			= GPIO_PinSource9;
		GPIO_PinSource_Rx 			= GPIO_PinSource14;
		GPIO_AF 					= GPIO_AF_USART6;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
		USARTx = USART6;
		NVIC_IRQChannel = USART6_IRQn;
	}
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph, ENABLE);
	GPIO_Init(GPIOx, &GPIOInitStructure);
	GPIO_PinAFConfig(GPIOx, GPIO_PinSource_Tx, GPIO_AF);
	GPIO_PinAFConfig(GPIOx, GPIO_PinSource_Rx, GPIO_AF);
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
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
	USART_ITConfig(USARTx, USART_IT_TC, ENABLE); // enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = NVIC_IRQChannel;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);


	USART_Cmd(USARTx, ENABLE);
	data->__drv_usart_base = (void*)USARTx;
	g_usart_driver_arch_data.rx_event[dev->id] = xSemaphoreCreateBinary();
	g_usart_driver_arch_data.tx_event[dev->id] = xSemaphoreCreateBinary();
	ret = 0;
	return ret;
}
int 	usart_close		(struct platform_device *dev){
	int ret = -EPERM;
	USART_TypeDef* USARTx;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;

	return 0;

	if(!dev || dev->id < 0 || dev->id >= USART_MODULE_COUNT) return ret;
	USART_Cmd((USART_TypeDef*)data->__drv_usart_base, DISABLE);
	ret = 0;
	return ret;
}
int		usart_read		(struct platform_device *dev, void* buf, int count){
	int ret = -EPERM;
	USART_TypeDef* USARTx;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;
	unsigned char* p = (unsigned char*)buf;

	if(!dev || dev->id < 0 || dev->id >= USART_MODULE_COUNT) return ret;

	ret = 0;
	while(count > 0){
		while((((USART_TypeDef*)data->__drv_usart_base)->SR & USART_SR_RXNE) == 0){
		}
		*p = USART_ReceiveData((USART_TypeDef*)data->__drv_usart_base);
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
//		while(!(((USART_TypeDef*)data->__drv_usart_base)->SR & USART_SR_TC)){}
		USART_SendData((USART_TypeDef*)data->__drv_usart_base, *p);
//		xSemaphoreTake(g_usart_driver_arch_data.tx_event[dev->id], 10);
		count --;
		p++;
		ret++;
	}
	return ret;
}
// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART1_IRQHandler(void){
	static BaseType_t xHigherPriorityTaskWoken;
	if( USART_GetITStatus(USART1, USART_IT_TC) ){
		USART_ClearITPendingBit(USART1, USART_IT_TC);
		xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(g_usart_driver_arch_data.tx_event[0], &xHigherPriorityTaskWoken);
				GPIO_ToggleBits(GPIOD,GPIO_Pin_14);


	}
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);

		xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(g_usart_driver_arch_data.rx_event[0], &xHigherPriorityTaskWoken);
	}
}
