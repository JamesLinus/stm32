#include <drv_api.h>
#include <drv_usart.h>
#include <drv_gpio.h>
#if defined(STDPERIPH_DRIVER)
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#elif defined(STM32CUBEF4)
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_gpio_ex.h>
#include <stm32f4xx_hal_uart.h>
#endif
#if defined(OS_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#elif defined(OS_UCOS)
#endif
#include <termios.h>
#include <ringbuffer.h>

#define DRV_UART_RX_BUF_SIZE	(32)

int 	usart_init		(void);
int 	usart_open		(struct platform_device *dev, int flags);
int 	usart_close		(struct platform_device *dev);
int		usart_write		(struct platform_device *dev, const void* buf, int count);
int		usart_read		(struct platform_device *dev, void* buf, int count);
int		usart_ioctl		(struct platform_device *dev, int request, unsigned int arguments);
int		usart_select	(struct platform_device *device, int *readfd, int *writefd, int *exceptfd, int timeout);
void 	USART1_IRQHandler(void);
void 	USART2_IRQHandler(void);
void 	USART3_IRQHandler(void);
void 	UART4_IRQHandler(void);
void 	UART5_IRQHandler(void);
void 	USART6_IRQHandler(void);
struct usart_driver_arch_data{
#if defined(OS_FREERTOS)
	void* rx_event[USART_MODULE_COUNT];
#elif defined(OS_UCOS)
	OS_FLAG_GRP rx_event;
	struct RingBuffer	rx_buf[USART_MODULE_COUNT];
	uint8_t				__rx_buf[USART_MODULE_COUNT][DRV_UART_RX_BUF_SIZE];
	UART_HandleTypeDef* handle[USART_MODULE_COUNT];
#endif
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
#if defined(OS_UCOS)
	OS_ERR err;
	int i;
	OSFlagCreate(&g_usart_driver_arch_data.rx_event, "", 0, &err);
	for(i = 0; i < USART_MODULE_COUNT; i++){
		RingBuffer_Init(&g_usart_driver_arch_data.rx_buf[i], DRV_UART_RX_BUF_SIZE, &g_usart_driver_arch_data.__rx_buf[i][0]);
	}
#endif
	platform_driver_register(&g_usart_driver);
	return 0;
}
module_init(usart_init);
int 	usart_open		(struct platform_device *dev, int flags){
	int ret = -EPERM;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;
	unsigned int bank, pin;	
	GPIO_TypeDef	 *GPIOx;
	GPIO_InitTypeDef  GPIOInitStructure;
#if defined(STDPERIPH_DRIVER)
	USART_TypeDef	 *USARTx;
	USART_InitTypeDef USART_InitStructure;
	uint32_t RCC_AHB1Periph;
	uint16_t GPIO_PinSource;
	uint8_t GPIO_AF;
	uint8_t NVIC_IRQChannel;
	NVIC_InitTypeDef NVIC_InitStructure;
#elif defined(STM32CUBEF4)
#endif
	if(!dev || dev->id < 0 || dev->id >= USART_MODULE_COUNT) return ret;
	if(dev->id == 0){
		// usart1
#if defined(STDPERIPH_DRIVER)
		GPIO_AF 		= GPIO_AF_USART1;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		NVIC_IRQChannel = USART1_IRQn;
#elif defined(STM32CUBEF4)
		GPIOInitStructure.Alternate = GPIO_AF7_USART1;
		__HAL_RCC_USART1_CLK_ENABLE();
		data->__drv_usart_base.Instance = USART1;
		BSP_IntVectSet(BSP_INT_ID_USART1, &USART1_IRQHandler);
		HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
#endif
	}else if(dev->id == 1){
		// usart2
#if defined(STDPERIPH_DRIVER)
		GPIO_AF 		= GPIO_AF_USART2;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		USARTx = USART2;
		NVIC_IRQChannel = USART2_IRQn;
#elif defined(STM32CUBEF4)
		GPIOInitStructure.Alternate = GPIO_AF7_USART2;
		__HAL_RCC_USART2_CLK_ENABLE();
		data->__drv_usart_base.Instance = USART2;
		BSP_IntVectSet(BSP_INT_ID_USART2, &USART2_IRQHandler);
		HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif
	}
	else if(dev->id == 2){
		// usart3
#if defined(STDPERIPH_DRIVER)
		GPIO_AF 		= GPIO_AF_USART3;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		USARTx = USART3;
		NVIC_IRQChannel = USART3_IRQn;
#elif defined(STM32CUBEF4)
		GPIOInitStructure.Alternate = GPIO_AF7_USART3;
		__HAL_RCC_USART3_CLK_ENABLE();
		data->__drv_usart_base.Instance = USART3;
		BSP_IntVectSet(BSP_INT_ID_USART3, &USART3_IRQHandler);
		HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
#endif
	}
	else if(dev->id == 3){
		// usart4
#if defined(STDPERIPH_DRIVER)
		GPIO_AF 		= GPIO_AF_UART4;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		USARTx = UART4;
		NVIC_IRQChannel = UART4_IRQn;
#elif defined(STM32CUBEF4)
		GPIOInitStructure.Alternate = GPIO_AF8_UART4;
		__HAL_RCC_UART4_CLK_ENABLE();
		data->__drv_usart_base.Instance = UART4;
		BSP_IntVectSet(BSP_INT_ID_USART4, &UART4_IRQHandler);
		HAL_NVIC_SetPriority(UART4_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(UART4_IRQn);
#endif
	}
	else if(dev->id == 4){
		//PC12
#if defined(STDPERIPH_DRIVER)
		GPIO_AF 		= GPIO_AF_UART5;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
		USARTx = UART5;
		NVIC_IRQChannel = UART5_IRQn;
#elif defined(STM32CUBEF4)
		GPIOInitStructure.Alternate = GPIO_AF8_UART5;
		__HAL_RCC_UART5_CLK_ENABLE();
		data->__drv_usart_base.Instance = UART5;
		BSP_IntVectSet(BSP_INT_ID_USART5, &UART5_IRQHandler);
		HAL_NVIC_SetPriority(UART5_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(UART5_IRQn);
#endif
	}
	else if(dev->id == 5){
		// usart6 PG14 PG9
#if defined(STDPERIPH_DRIVER)
		GPIO_AF 		= GPIO_AF_USART6;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
		USARTx = USART6;
		NVIC_IRQChannel = USART6_IRQn;
		data->__drv_usart_base.Instance = USART6;
#elif defined(STM32CUBEF4)
		GPIOInitStructure.Alternate = GPIO_AF8_USART6;
		__HAL_RCC_USART6_CLK_ENABLE();
		data->__drv_usart_base.Instance = USART6;
		BSP_IntVectSet(BSP_INT_ID_USART6, &USART6_IRQHandler);
		HAL_NVIC_SetPriority(USART6_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(USART6_IRQn);
#endif
	}
	// config gpio
#if defined(STDPERIPH_DRIVER)
	GPIOInitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIOInitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIOInitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIOInitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
#elif defined(STM32CUBEF4)
	GPIOInitStructure.Mode			= GPIO_MODE_AF_PP;
	GPIOInitStructure.Pull			= GPIO_NOPULL;
	GPIOInitStructure.Speed			= GPIO_SPEED_FAST;
#endif
	// rx
	bank = gpio_get_bank_index(data->rx_pin);
	pin  = gpio_get_pin_index(data->rx_pin);
#if defined(STDPERIPH_DRIVER)
	RCC_AHB1PeriphClockCmd(g_gpio_bank_ref[bank].RCC_AHB1Periph_GPIOx, ENABLE);
	GPIOInitStructure.GPIO_Pin 	= g_gpio_pin_ref[pin].GPIO_Pin;
	GPIO_PinAFConfig(g_gpio_bank_ref[bank].GPIOx, g_gpio_pin_ref[pin].GPIO_PinSource, GPIO_AF);
	GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIOInitStructure);
#elif defined(STM32CUBEF4)
	switch(bank){
		case 0: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
		case 1: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
		case 2: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
		case 3: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
		case 4: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
		case 5: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
		case 6: __HAL_RCC_GPIOG_CLK_ENABLE(); break;
		case 7: __HAL_RCC_GPIOH_CLK_ENABLE(); break;
		case 8: __HAL_RCC_GPIOI_CLK_ENABLE(); break;
		default: break;
	}
	GPIOInitStructure.Pin 	= g_gpio_pin_ref[pin].GPIO_Pin;
	HAL_GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIOInitStructure);
#endif
	// tx
	bank = gpio_get_bank_index(data->tx_pin);
	pin  = gpio_get_pin_index(data->tx_pin);
#if defined(STDPERIPH_DRIVER)
	RCC_AHB1PeriphClockCmd(g_gpio_bank_ref[bank].RCC_AHB1Periph_GPIOx, ENABLE);
	GPIOInitStructure.GPIO_Pin 	= g_gpio_pin_ref[pin].GPIO_Pin;
	GPIO_PinAFConfig(g_gpio_bank_ref[bank].GPIOx, g_gpio_pin_ref[pin].GPIO_PinSource, GPIO_AF);
	GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIOInitStructure);
#elif defined(STM32CUBEF4)
	switch(bank){
		case 0: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
		case 1: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
		case 2: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
		case 3: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
		case 4: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
		case 5: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
		case 6: __HAL_RCC_GPIOG_CLK_ENABLE(); break;
		case 7: __HAL_RCC_GPIOH_CLK_ENABLE(); break;
		case 8: __HAL_RCC_GPIOI_CLK_ENABLE(); break;
		default: break;
	}
	GPIOInitStructure.Pin 	= g_gpio_pin_ref[pin].GPIO_Pin;
	HAL_GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIOInitStructure);
#endif

	// config usart	
	/**uart1 configured as follow
	* baudrate 9600 baud
	* word length 8 bits
	* 1 stop bit
	* no parity
	* hardware flow control disabled
	* receive and transmit enable
	*/
#if defined(STDPERIPH_DRIVER)
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
#elif defined(STM32CUBEF4)
	data->__drv_usart_base.Init.BaudRate	= 9600;
	data->__drv_usart_base.Init.WordLength	= UART_WORDLENGTH_8B;
	data->__drv_usart_base.Init.StopBits	= UART_STOPBITS_1;
	data->__drv_usart_base.Init.Parity		= UART_PARITY_NONE;
	data->__drv_usart_base.Init.Mode		= UART_MODE_TX_RX;
	data->__drv_usart_base.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	data->__drv_usart_base.Init.OverSampling = UART_OVERSAMPLING_16;
	g_usart_driver_arch_data.handle[dev->id] = &data->__drv_usart_base;
	HAL_UART_Init(g_usart_driver_arch_data.handle[dev->id]);
	__HAL_UART_ENABLE_IT(g_usart_driver_arch_data.handle[dev->id], UART_IT_RXNE);
#endif
#if defined(OS_FREERTOS)
	if(!g_usart_driver_arch_data.rx_event[dev->id])
		g_usart_driver_arch_data.rx_event[dev->id] = xQueueCreate(32, 1);
#endif

	ret = 0;
	return ret;
}
int 	usart_close		(struct platform_device *dev){
	int ret = -EPERM;
	USART_TypeDef* USARTx;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;

	if(!dev || dev->id < 0 || dev->id >= USART_MODULE_COUNT) return ret;
#if defined(STDPERIPH_DRIVER)
	USART_Cmd((USART_TypeDef*)data->__drv_usart_base, DISABLE);
#elif defined(STM32CUBEF4)
	HAL_UART_DeInit(&data->__drv_usart_base);
#endif
	ret = 0;
	return ret;
}
int		usart_read		(struct platform_device *dev, void* buf, int count){
	int ret = -EPERM;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;
	unsigned char* p = (unsigned char*)buf;
#if defined(OS_UCOS)
	OS_ERR err;
	uint8_t* rx;
	OS_MSG_SIZE p_msg_size;
#endif

	if(!dev || dev->id < 0 || dev->id >= USART_MODULE_COUNT) return ret;
#if defined(OS_FREERTOS)
	ret = 0;
	while(count > 0){
		if(xQueueReceive(g_usart_driver_arch_data.rx_event[dev->id], p, 0) != pdTRUE)
			break;
		count --;
		p++;
		ret++;
	}
#elif defined(OS_UCOS)
	ret = RingBuffer_Pop(&g_usart_driver_arch_data.rx_buf[dev->id], p, count);
#endif
	return ret;
}
int		usart_write	(struct platform_device *dev, const void* buf, int count){
	int ret = -EPERM;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;
	unsigned char* p = (unsigned char*)buf;
	if(!dev || dev->id < 0 || dev->id >= USART_MODULE_COUNT) return ret;

	ret = 0;
#if defined(STM32CUBEF4)
	uint32_t timeout = TICK_RATE_HZ;
	HAL_UART_Transmit(&data->__drv_usart_base, p, count, timeout);
#elif defined(STDPERIPH_DRIVER)
	while(count > 0){
		while(!(((USART_TypeDef*)data->__drv_usart_base)->SR & USART_SR_TC)){}
		USART_SendData((USART_TypeDef*)data->__drv_usart_base, *p);
		count --;
		p++;
		ret++;
	}
#endif
	return ret;
}
int		usart_ioctl	(struct platform_device *dev, int request, unsigned int arguments){
	int ret = -EPERM;
	struct termios2* opt = 0;
	UART_InitTypeDef USART_InitStructure;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;
	
	switch(request){
		case TCSETS2:{
			opt = (struct termios2*)arguments;
#if defined(STDPERIPH_DRIVER)
			USART_InitStructure.USART_BaudRate		= opt->c_ispeed;
			USART_InitStructure.USART_WordLength	= USART_WordLength_8b;
			USART_InitStructure.USART_StopBits 		= USART_StopBits_1;
			USART_InitStructure.USART_Parity 		= USART_Parity_No;
			USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode 			= USART_Mode_Rx | USART_Mode_Tx;
			USART_Init((USART_TypeDef*)data->__drv_usart_base, &USART_InitStructure);
#elif defined(STM32CUBEF4)
			data->__drv_usart_base.Init.BaudRate	= opt->c_ispeed;
			data->__drv_usart_base.Init.WordLength	= UART_WORDLENGTH_8B;
			data->__drv_usart_base.Init.StopBits	= UART_STOPBITS_1;
			data->__drv_usart_base.Init.Parity		= UART_PARITY_NONE;
			data->__drv_usart_base.Init.Mode		= UART_MODE_TX_RX;
			data->__drv_usart_base.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
			data->__drv_usart_base.Init.OverSampling = UART_OVERSAMPLING_16;
			HAL_UART_Init(&data->__drv_usart_base);
#endif
			ret = 0;
			break;
		}
		default:
			break;
	}
	
	return ret;
}
#include <debug.h>

int		usart_select(struct platform_device *dev, int *readfd, int *writefd, int *exceptfd, int timeout){
	int ret = -EPERM;
	uint8_t u8data;
	struct usart_platform_data* data = (struct usart_platform_data*)dev->dev.platform_data;
	if(readfd) 		*readfd = 0;
	if(writefd) 	*writefd = 0;
	if(exceptfd) 	*exceptfd = 0;
	if(readfd){
#if defined(OS_UCOS)
		OS_FLAGS flags;
		OS_ERR err;
		flags = OSFlagPend(&g_usart_driver_arch_data.rx_event,
				((OS_FLAGS)1) << dev->id,
				timeout,
				OS_OPT_PEND_FLAG_SET_ANY|OS_OPT_PEND_FLAG_CONSUME,
				0,
				&err);
		if(flags & (((OS_FLAGS)1) << dev->id)){
			*readfd = 1;
			ret = 1;
		}else{
			ret = 0;
		}
#elif defined(OS_FREERTOS)
		ret = xQueuePeek(g_usart_driver_arch_data.rx_event[dev->id], &u8data, timeout);
		if(ret == pdTRUE) {
			*readfd = 1;
			ret = 1;
		}
		else ret = 0;
#endif
	}	
	return ret;
}
// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART1_IRQHandler(void){
	static uint8_t data;
#if defined(STDPERIPH_DRIVER)
	static BaseType_t xHigherPriorityTaskWoken;
	static USART_TypeDef* USARTx = USART1;
	
	if( USART_GetITStatus(USARTx, USART_IT_RXNE) ){
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		data = USART_ReceiveData(USARTx);
		xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(g_usart_driver_arch_data.rx_event[0], &data, &xHigherPriorityTaskWoken);
	}
#elif defined(STM32CUBEF4)
	OS_ERR p_err;
	if(__HAL_UART_GET_FLAG(g_usart_driver_arch_data.handle[0], UART_FLAG_RXNE)){
		__HAL_UART_CLEAR_FLAG(g_usart_driver_arch_data.handle[0], UART_FLAG_RXNE);
		data = g_usart_driver_arch_data.handle[0]->Instance->DR & (uint8_t)0x00FF;
		RingBuffer_Push(&g_usart_driver_arch_data.rx_buf[0], &data, 1);
		OSFlagPost(&g_usart_driver_arch_data.rx_event, ((OS_FLAGS)1) << 0, OS_OPT_POST_FLAG_SET, &p_err);
	}
#endif
}
void USART2_IRQHandler(void){
	static uint8_t data;
#if defined(STDPERIPH_DRIVER)
	static uint8_t data;
	static BaseType_t xHigherPriorityTaskWoken;
	static USART_TypeDef* USARTx = USART2;
	
	if( USART_GetITStatus(USARTx, USART_IT_RXNE) ){
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		data = USART_ReceiveData(USARTx);
		xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(g_usart_driver_arch_data.rx_event[1], &data, &xHigherPriorityTaskWoken);
	}
#elif defined(STM32CUBEF4)
	OS_ERR p_err;
	if(__HAL_UART_GET_FLAG(g_usart_driver_arch_data.handle[1], UART_FLAG_RXNE)){
		__HAL_UART_CLEAR_FLAG(g_usart_driver_arch_data.handle[1], UART_FLAG_RXNE);
		data = g_usart_driver_arch_data.handle[1]->Instance->DR & (uint8_t)0x00FF;
		RingBuffer_Push(&g_usart_driver_arch_data.rx_buf[1], &data, 1);
		OSFlagPost(&g_usart_driver_arch_data.rx_event, ((OS_FLAGS)1) << 1, OS_OPT_POST_FLAG_SET, &p_err);
	}
#endif
}
void USART3_IRQHandler(void){
	static uint8_t data;
#if defined(STDPERIPH_DRIVER)
	static uint8_t data;
	static BaseType_t xHigherPriorityTaskWoken;
	static USART_TypeDef* USARTx = USART3;
	
	if( USART_GetITStatus(USARTx, USART_IT_RXNE) ){
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		data = USART_ReceiveData(USARTx);
		xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(g_usart_driver_arch_data.rx_event[2], &data, &xHigherPriorityTaskWoken);
	}
#elif defined(STM32CUBEF4)
	OS_ERR p_err;
	if(__HAL_UART_GET_FLAG(g_usart_driver_arch_data.handle[2], UART_FLAG_RXNE)){
		__HAL_UART_CLEAR_FLAG(g_usart_driver_arch_data.handle[2], UART_FLAG_RXNE);
		data = g_usart_driver_arch_data.handle[2]->Instance->DR & (uint8_t)0x00FF;
		RingBuffer_Push(&g_usart_driver_arch_data.rx_buf[2], &data, 1);
		OSFlagPost(&g_usart_driver_arch_data.rx_event, ((OS_FLAGS)1) << 2, OS_OPT_POST_FLAG_SET, &p_err);
	}
#endif
}
void UART4_IRQHandler(void){
	static uint8_t data;
#if defined(STDPERIPH_DRIVER)
	static uint8_t data;
	static BaseType_t xHigherPriorityTaskWoken;
	static USART_TypeDef* USARTx = UART4;
	
	if( USART_GetITStatus(USARTx, USART_IT_RXNE) ){
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		data = USART_ReceiveData(USARTx);
		xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(g_usart_driver_arch_data.rx_event[3], &data, &xHigherPriorityTaskWoken);
	}
#elif defined(STM32CUBEF4)
	OS_ERR p_err;
	if(__HAL_UART_GET_FLAG(g_usart_driver_arch_data.handle[3], UART_FLAG_RXNE)){
		__HAL_UART_CLEAR_FLAG(g_usart_driver_arch_data.handle[3], UART_FLAG_RXNE);
		data = g_usart_driver_arch_data.handle[3]->Instance->DR & (uint8_t)0x00FF;
		RingBuffer_Push(&g_usart_driver_arch_data.rx_buf[3], &data, 1);
		OSFlagPost(&g_usart_driver_arch_data.rx_event, ((OS_FLAGS)1) << 3, OS_OPT_POST_FLAG_SET, &p_err);
	}
#endif
}
void UART5_IRQHandler(void){
	static uint8_t data;
#if defined(STDPERIPH_DRIVER)
	static uint8_t data;
	static BaseType_t xHigherPriorityTaskWoken;
	static USART_TypeDef* USARTx = UART5;
	
	if( USART_GetITStatus(USARTx, USART_IT_RXNE) ){
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		data = USART_ReceiveData(USARTx);
		xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(g_usart_driver_arch_data.rx_event[4], &data, &xHigherPriorityTaskWoken);
	}
#elif defined(STM32CUBEF4)
	OS_ERR p_err;
	if(__HAL_UART_GET_FLAG(g_usart_driver_arch_data.handle[4], UART_FLAG_RXNE)){
		__HAL_UART_CLEAR_FLAG(g_usart_driver_arch_data.handle[4], UART_FLAG_RXNE);
		data = g_usart_driver_arch_data.handle[4]->Instance->DR & (uint8_t)0x00FF;
		RingBuffer_Push(&g_usart_driver_arch_data.rx_buf[4], &data, 1);
		OSFlagPost(&g_usart_driver_arch_data.rx_event, ((OS_FLAGS)1) << 4, OS_OPT_POST_FLAG_SET, &p_err);
	}
#endif
}
void USART6_IRQHandler(void){
	static uint8_t data;
#if defined(STDPERIPH_DRIVER)
	static uint8_t data;
	static BaseType_t xHigherPriorityTaskWoken;
	static USART_TypeDef* USARTx = USART6;
	
	if( USART_GetITStatus(USARTx, USART_IT_RXNE) ){
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		data = USART_ReceiveData(USARTx);
		xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(g_usart_driver_arch_data.rx_event[5], &data, &xHigherPriorityTaskWoken);
	}
#elif defined(STM32CUBEF4)
	OS_ERR p_err;
	if(__HAL_UART_GET_FLAG(g_usart_driver_arch_data.handle[5], UART_FLAG_RXNE)){
		__HAL_UART_CLEAR_FLAG(g_usart_driver_arch_data.handle[5], UART_FLAG_RXNE);
		data = g_usart_driver_arch_data.handle[5]->Instance->DR & (uint8_t)0x00FF;
		RingBuffer_Push(&g_usart_driver_arch_data.rx_buf[5], &data, 1);
		OSFlagPost(&g_usart_driver_arch_data.rx_event, ((OS_FLAGS)1) << 5, OS_OPT_POST_FLAG_SET, &p_err);
	}
#endif
}



