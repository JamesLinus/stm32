#include "../include/drv_api.h"
#include "../include/drv_gpio.h"

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

int 	gpio_init		(void);
int 	gpio_open		(struct platform_device *dev, int flags);
int 	gpio_close		(struct platform_device *dev);
int		gpio_write		(struct platform_device *dev, const void* buf, int count);
int		gpio_read		(struct platform_device *dev, void* buf, int count);
int		gpio_ioctl		(struct platform_device *dev, int request, unsigned int arguments);
int		gpio_select	(struct platform_device *device, int *readfd, int *writefd, int *exceptfd, int timeout);

struct gpio_driver_arch_data{
	EventGroupHandle_t	event;
};
struct gpio_driver_arch_data g_gpio_driver_arch_data;

static struct platform_driver g_gpio_driver = {
	.driver		= {
		.name	= "gpio-drv",
		.devices = 0,
	},
	.archdata = &g_gpio_driver_arch_data,
	.probe		= 0,
	.remove		= 0,
	.resume 	= 0,
	.suspend	= 0,
	.shutdown	= 0,

	.open 		= &gpio_open,
	.close 		= &gpio_close,
	.read 		= &gpio_read,
	.write 		= &gpio_write,
	.ioctl 		= &gpio_ioctl,
	.select		= &gpio_select,

	.next 		= 0,
};

int gpio_init		(void){
	g_gpio_driver_arch_data.event = xEventGroupCreate();
	platform_driver_register(&g_gpio_driver);
	return 0;
}
module_init(gpio_init);
// pin = bank_index * 32 + pin_index
// bank_id:
//   A-0, B-1, C-2, D-3, E-4, F-5, G-6, H-7, I-8
inline int gpio_get_bank_index(int pin_id){
	return pin_id / GPIO_PIN_COUNT;
}
inline int gpio_get_pin_index(int pin_id){
	return pin_id % GPIO_PIN_COUNT;
}
struct gpio_param_ref {
	GPIO_TypeDef* 	GPIOx;
	uint32_t 		RCC_AHB1Periph_GPIOx;
	uint32_t		EXTI_PortSourceGPIOx;
};
struct gpio_pin_ref {
	uint8_t NVIC_IRQChannel;
};
struct gpio_pin_ref g_gpio_pin_ref[] = {
	{
		.NVIC_IRQChannel = EXTI0_IRQn,
	},
	{
		.NVIC_IRQChannel = EXTI1_IRQn,
	},
	{
		.NVIC_IRQChannel = EXTI2_IRQn,
	},
	{
		.NVIC_IRQChannel = EXTI3_IRQn,
	},
	{
		.NVIC_IRQChannel = EXTI4_IRQn,
	},
	{
		.NVIC_IRQChannel = EXTI9_5_IRQn,
	},
	{
		.NVIC_IRQChannel = EXTI9_5_IRQn,
	},
	{
		.NVIC_IRQChannel = EXTI9_5_IRQn,
	},
	{
		.NVIC_IRQChannel = EXTI9_5_IRQn,
	},
	{
		.NVIC_IRQChannel = EXTI9_5_IRQn,
	},
	{
		.NVIC_IRQChannel = EXTI15_10_IRQn,
	},
	{
		.NVIC_IRQChannel = EXTI15_10_IRQn,
	},
	{
		.NVIC_IRQChannel = EXTI15_10_IRQn,
	},
	{
		.NVIC_IRQChannel = EXTI15_10_IRQn,
	},
	{
		.NVIC_IRQChannel = EXTI15_10_IRQn,
	},
	{
		.NVIC_IRQChannel = EXTI15_10_IRQn,
	},
};
struct gpio_param_ref g_gpio_param_ref[] = {
	{
		.GPIOx = GPIOA,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOA,
		.EXTI_PortSourceGPIOx = EXTI_PortSourceGPIOA,
	},
		{
		.GPIOx = GPIOB,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOB,
		.EXTI_PortSourceGPIOx = EXTI_PortSourceGPIOB,
	},
		{
		.GPIOx = GPIOC,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOC,
		.EXTI_PortSourceGPIOx = EXTI_PortSourceGPIOC,
	},
		{
		.GPIOx = GPIOD,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOD,
		.EXTI_PortSourceGPIOx = EXTI_PortSourceGPIOD,
	},
		{
		.GPIOx = GPIOE,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOE,
		.EXTI_PortSourceGPIOx = EXTI_PortSourceGPIOE,
	},
		{
		.GPIOx = GPIOF,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOF,
		.EXTI_PortSourceGPIOx = EXTI_PortSourceGPIOF,
	},
		{
		.GPIOx = GPIOG,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOG,
		.EXTI_PortSourceGPIOx = EXTI_PortSourceGPIOG,
	},
		{
		.GPIOx = GPIOH,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOH,
		.EXTI_PortSourceGPIOx = EXTI_PortSourceGPIOH,
	},
		{
		.GPIOx = GPIOI,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOI,
		.EXTI_PortSourceGPIOx = EXTI_PortSourceGPIOI,
	},
};
//extern void LREP(char* s, ...);
int 	gpio_open	(struct platform_device *dev, int flags){
	int ret = -EPERM;
	int bank = gpio_get_bank_index(dev->id);
	int pin = gpio_get_pin_index(dev->id);
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	struct gpio_platform_data* data = (struct gpio_platform_data*)dev->dev.platform_data;
	
	if(bank < 0 || bank >= GPIO_BANK_COUNT ||
		pin < 0 || pin >= GPIO_PIN_COUNT)
		return ret;
	
	RCC_AHB1PeriphClockCmd(g_gpio_param_ref[bank].RCC_AHB1Periph_GPIOx, ENABLE);
	GPIO_InitStructure.GPIO_Mode 	= data->dir;
	GPIO_InitStructure.GPIO_Pin 	= (((uint16_t)0x01) << pin);
	GPIO_InitStructure.GPIO_PuPd 	= data->pull;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_Init(g_gpio_param_ref[bank].GPIOx, &GPIO_InitStructure);
	
	if(data->intr.mode != GPIO_INTR_MODE_DISABLE){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		
		SYSCFG_EXTILineConfig(g_gpio_param_ref[bank].EXTI_PortSourceGPIOx, pin);
		EXTI_InitStruct.EXTI_Line 		= (((uint32_t)1) << pin);
		EXTI_InitStruct.EXTI_LineCmd 	= ENABLE;
		EXTI_InitStruct.EXTI_Mode 		= data->intr.mode;
		EXTI_InitStruct.EXTI_Trigger 	= data->intr.trigger;
		EXTI_Init(&EXTI_InitStruct);

		/* Add IRQ vector to NVIC */
		NVIC_InitStruct.NVIC_IRQChannel 	= g_gpio_pin_ref[pin].NVIC_IRQChannel;
		NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority 	= 6;
		NVIC_InitStruct.NVIC_IRQChannelSubPriority 			= 0;
		NVIC_InitStruct.NVIC_IRQChannelCmd 	= ENABLE;
		NVIC_Init(&NVIC_InitStruct);		 
	}
	ret = 0;	
	return ret;
}
int 	gpio_close	(struct platform_device *dev){
	int ret = -EPERM;
	return ret;
}
int		gpio_read	(struct platform_device *dev, void* buf, int count){
	int ret = -EPERM;
	struct gpio_platform_data* data = (struct gpio_platform_data*)dev->dev.platform_data;
	unsigned char* p = (unsigned char*)buf;
	int bank = gpio_get_bank_index(dev->id);
	int pin = gpio_get_pin_index(dev->id);
	
	if(count > 0){
		*p = GPIO_ReadInputDataBit(g_gpio_param_ref[bank].GPIOx, (((uint16_t)0x01)<< pin));
		ret = 1;
	}
	return ret;
}
int		gpio_write	(struct platform_device *dev, const void* buf, int count){
	int ret = -EPERM;
	struct gpio_platform_data* data = (struct gpio_platform_data*)dev->dev.platform_data;
	unsigned char* p = (unsigned char*)buf;
	int bank = gpio_get_bank_index(dev->id);
	int pin = gpio_get_pin_index(dev->id);
	
	if(count > 0){
		if(p[0] == 0) GPIO_ResetBits(g_gpio_param_ref[bank].GPIOx, (((uint16_t)0x01)<< pin));
		else GPIO_SetBits(g_gpio_param_ref[bank].GPIOx, (((uint16_t)0x01)<< pin));
		ret = 0;
	}	
		
	return ret;
}
int		gpio_ioctl	(struct platform_device *dev, int request, unsigned int arguments){
	int ret = -EPERM;
	
	return ret;
}
int		gpio_select(struct platform_device *dev, int *readfd, int *writefd, int *exceptfd, int timeout){
	int ret = -EPERM;
	EventBits_t uxBits;
	uint8_t u8data;
	int bank = gpio_get_bank_index(dev->id);
	int pin = gpio_get_pin_index(dev->id);
	struct gpio_platform_data* data = (struct gpio_platform_data*)dev->dev.platform_data;
	
	if(readfd) 		*readfd = 0;
	if(writefd) 	*writefd = 0;
	if(exceptfd) 	*exceptfd = 0;
	if(readfd){
		uxBits = xEventGroupWaitBits(g_gpio_driver_arch_data.event,
				((uint32_t)1) << pin,
				pdTRUE,
				pdFALSE,
				timeout);
		if(uxBits & (((uint32_t)1) << pin)) {
			*readfd = 1;
			ret = 1;
		}
		else ret = 0;
	}	
	return ret;
}
void EXTI0_IRQHandler(void) {
	static BaseType_t xHigherPriorityTaskWoken, xResult;

	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 0),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line0);
		/* Was the message posted successfully? */
		if( xResult != pdFAIL )
		{
		  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
		  switch should be requested.  The macro used is port specific and will
		  be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
		  the documentation page for the port being used. */
		  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}
    }
}
void EXTI1_IRQHandler(void) {
	static BaseType_t xHigherPriorityTaskWoken, xResult;

	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 1),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line1);
		/* Was the message posted successfully? */
		if( xResult != pdFAIL )
		{
		  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
		  switch should be requested.  The macro used is port specific and will
		  be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
		  the documentation page for the port being used. */
		  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}
    }
}
void EXTI2_IRQHandler(void) {
	static BaseType_t xHigherPriorityTaskWoken, xResult;

	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 2),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line2);
		/* Was the message posted successfully? */
		if( xResult != pdFAIL )
		{
		  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
		  switch should be requested.  The macro used is port specific and will
		  be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
		  the documentation page for the port being used. */
		  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}
    }
}
void EXTI3_IRQHandler(void) {
	static BaseType_t xHigherPriorityTaskWoken, xResult;

	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 3),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line3);
		/* Was the message posted successfully? */
		if( xResult != pdFAIL )
		{
		  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
		  switch should be requested.  The macro used is port specific and will
		  be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
		  the documentation page for the port being used. */
		  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}
    }
}
void EXTI4_IRQHandler(void) {
	static BaseType_t xHigherPriorityTaskWoken, xResult;

	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line4) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 4),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line4);
		/* Was the message posted successfully? */
		if( xResult != pdFAIL )
		{
		  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
		  switch should be requested.  The macro used is port specific and will
		  be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
		  the documentation page for the port being used. */
		  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}
    }
}
void EXTI9_5_IRQHandler(void) {
	static BaseType_t xHigherPriorityTaskWoken, xResult;
	xResult = pdFAIL;

	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 5),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line5);
    }
    if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 6),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line6);
    }
    if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 7),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line7);
    }
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 8),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line8);
    }
    if (EXTI_GetITStatus(EXTI_Line9) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 9),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line9);
    }
	/* Was the message posted successfully? */
	if( xResult != pdFAIL )
	{
	  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
	  switch should be requested.  The macro used is port specific and will
	  be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
	  the documentation page for the port being used. */
	  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}
void EXTI15_10_IRQHandler(void) {
	static BaseType_t xHigherPriorityTaskWoken, xResult;
	xResult = pdFAIL;
	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 10),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line10);
    }
    if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 11),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line11);
    }
    if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 12),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line12);
    }
    if (EXTI_GetITStatus(EXTI_Line13) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 13),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line13);
    }
    if (EXTI_GetITStatus(EXTI_Line14) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 14),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line14);
    }
    if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = xEventGroupSetBitsFromISR(
				g_gpio_driver_arch_data.event, 
				(((uint8_t)1)<< 15),
				&xHigherPriorityTaskWoken);
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line15);
    }
    /* Was the message posted successfully? */
	if( xResult != pdFAIL )
	{
	  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
	  switch should be requested.  The macro used is port specific and will
	  be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
	  the documentation page for the port being used. */
	  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}
// end of file
