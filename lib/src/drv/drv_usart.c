#include "../include/drv_api.h"
#include "../include/drv_usart.h"

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"

int 	usart_init		(void);
int 	usart_open		(struct platform_device *dev, int flags);
int 	usart_close		(struct platform_device *dev);
int		usart_write		(struct platform_device *dev, const void* buf, int count);

static struct platform_driver g_usart_driver = {
	.driver		= {
		.name	= "usart-drv",
		.devices = 0,
	},
	.probe		= 0,
	.remove		= 0,
	.resume 	= 0,
	.suspend	= 0,
	.shutdown	= 0,

	.open 		= &usart_open,
	.close 		= &usart_close,
	.read 		= 0,
	.write 		= &usart_write,
	.ioctl 		= 0,

	.next 		= 0,
};
int usart_init		(void){

	platform_driver_register(&g_usart_driver);
	return 0;
}
module_init(usart_init);
int 	usart_open		(struct platform_device *dev, int flags){
	int ret = -EPERM;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;

	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef  GPIOInitStructure;
	uint32_t RCC_AHB1Periph;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_PinSource_Tx, GPIO_PinSource_Rx;
	uint8_t GPIO_AF;
	USART_TypeDef* USARTx;

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
	USART_Cmd(USARTx, ENABLE);
	data->__drv_usart_base = (void*)USARTx;
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
int		usart_read		(struct platform_device *dev, void* buf, int count){
	int ret = -EPERM;
	USART_TypeDef* USARTx;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;

	if(!dev || dev->id < 0 || dev->id >= USART_MODULE_COUNT) return ret;



	ret = 0;
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
		while(!(((USART_TypeDef*)data->__drv_usart_base)->SR & 0x00000040));
		USART_SendData((USART_TypeDef*)data->__drv_usart_base, *p);
		count --;
		p++;
		ret++;
	}
	return ret;
}
