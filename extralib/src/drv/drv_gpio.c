#include "../include/drv_api.h"
#include "../include/drv_gpio.h"
int 	gpio_init		(void);
int 	gpio_open		(struct platform_device *dev, int flags);
int 	gpio_close		(struct platform_device *dev);
int		gpio_write		(struct platform_device *dev, const void* buf, int count);
int		gpio_read		(struct platform_device *dev, void* buf, int count);
int		gpio_ioctl		(struct platform_device *dev, int request, unsigned int arguments);
int		gpio_select	(struct platform_device *device, int *readfd, int *writefd, int *exceptfd, int timeout);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
struct gpio_driver_arch_data{
#if defined(OS_FREERTOS)
	EventGroupHandle_t	event;
#elif defined(OS_UCOS)
	OS_FLAG_GRP			event;
#endif
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
#if defined(OS_FREERTOS)
	g_gpio_driver_arch_data.event = xEventGroupCreate();
#elif defined(OS_UCOS)
	OS_ERR os_err;
	OSFlagCreate(&g_gpio_driver_arch_data.event,
	                 "gpio event",
	                  0,
	                 &os_err);
#endif
	platform_driver_register(&g_gpio_driver);
	return 0;
}
module_init(gpio_init);
struct gpio_pin_ref g_gpio_pin_ref[] = {
#if defined(STDPERIPH_DRIVER)
	{
		.GPIO_Pin = GPIO_Pin_0,
		.GPIO_PinSource = GPIO_PinSource0,
		.NVIC_IRQChannel = EXTI0_IRQn,
	},
	{
		.GPIO_Pin = GPIO_Pin_1,
		.GPIO_PinSource = GPIO_PinSource1,
		.NVIC_IRQChannel = EXTI1_IRQn,
	},
	{
		.GPIO_Pin = GPIO_Pin_2,
		.GPIO_PinSource = GPIO_PinSource2,
		.NVIC_IRQChannel = EXTI2_IRQn,
	},
	{
		.GPIO_Pin = GPIO_Pin_3,
		.GPIO_PinSource = GPIO_PinSource3,
		.NVIC_IRQChannel = EXTI3_IRQn,
	},
	{
		.GPIO_Pin = GPIO_Pin_4,
		.GPIO_PinSource = GPIO_PinSource4,
		.NVIC_IRQChannel = EXTI4_IRQn,
	},
	{
		.GPIO_Pin = GPIO_Pin_5,
		.GPIO_PinSource = GPIO_PinSource5,
		.NVIC_IRQChannel = EXTI9_5_IRQn,
	},
	{
		.GPIO_Pin = GPIO_Pin_6,
		.GPIO_PinSource = GPIO_PinSource6,
		.NVIC_IRQChannel = EXTI9_5_IRQn,
	},
	{
		.GPIO_Pin = GPIO_Pin_7,
		.GPIO_PinSource = GPIO_PinSource7,
		.NVIC_IRQChannel = EXTI9_5_IRQn,
	},
	{
		.GPIO_Pin = GPIO_Pin_8,
		.GPIO_PinSource = GPIO_PinSource8,
		.NVIC_IRQChannel = EXTI9_5_IRQn,
	},
	{
		.GPIO_Pin = GPIO_Pin_9,
		.GPIO_PinSource = GPIO_PinSource9,
		.NVIC_IRQChannel = EXTI9_5_IRQn,
	},
	{
		.GPIO_Pin = GPIO_Pin_10,
		.GPIO_PinSource = GPIO_PinSource10,
		.NVIC_IRQChannel = EXTI15_10_IRQn,
	},
	{
		.GPIO_Pin = GPIO_Pin_11,
		.GPIO_PinSource = GPIO_PinSource11,
		.NVIC_IRQChannel = EXTI15_10_IRQn,
	},
	{
		.GPIO_Pin = GPIO_Pin_12,
		.GPIO_PinSource = GPIO_PinSource12,
		.NVIC_IRQChannel = EXTI15_10_IRQn,
	},
	{
		.GPIO_Pin = GPIO_Pin_13,
		.GPIO_PinSource = GPIO_PinSource13,
		.NVIC_IRQChannel = EXTI15_10_IRQn,
	},
	{
		.GPIO_Pin = GPIO_Pin_14,
		.GPIO_PinSource = GPIO_PinSource14,
		.NVIC_IRQChannel = EXTI15_10_IRQn,
	},
	{
		.GPIO_Pin = GPIO_Pin_15,
		.GPIO_PinSource = GPIO_PinSource15,	
		.NVIC_IRQChannel = EXTI15_10_IRQn,
	},
#elif defined(STM32CUBEF4)
	{
		.GPIO_Pin = GPIO_PIN_0,
		.IRQChannel = BSP_INT_ID_EXTI0,
		.isr = &EXTI0_IRQHandler,
		.irqType = EXTI0_IRQn,
	},
	{
		.GPIO_Pin = GPIO_PIN_1,
		.IRQChannel = BSP_INT_ID_EXTI1,
		.isr = &EXTI1_IRQHandler,
		.irqType = EXTI1_IRQn,
	},
	{
		.GPIO_Pin = GPIO_PIN_2,
		.IRQChannel = BSP_INT_ID_EXTI2,
		.isr = &EXTI2_IRQHandler,
		.irqType = EXTI2_IRQn,
	},
	{
		.GPIO_Pin = GPIO_PIN_3,
		.IRQChannel = BSP_INT_ID_EXTI3,
		.isr = &EXTI3_IRQHandler,
		.irqType = EXTI3_IRQn,
	},
	{
		.GPIO_Pin = GPIO_PIN_4,
		.IRQChannel = BSP_INT_ID_EXTI4,
		.isr = &EXTI4_IRQHandler,
		.irqType = EXTI4_IRQn,
	},
	{
		.GPIO_Pin = GPIO_PIN_5,
		.IRQChannel = BSP_INT_ID_EXTI9_5,
		.isr = &EXTI9_5_IRQHandler,
		.irqType = EXTI9_5_IRQn,
	},
	{
		.GPIO_Pin = GPIO_PIN_6,
		.IRQChannel = BSP_INT_ID_EXTI9_5,
		.isr = &EXTI9_5_IRQHandler,
		.irqType = EXTI9_5_IRQn,
	},
	{
		.GPIO_Pin = GPIO_PIN_7,
		.IRQChannel = BSP_INT_ID_EXTI9_5,
		.isr = &EXTI9_5_IRQHandler,
		.irqType = EXTI9_5_IRQn,
	},
	{
		.GPIO_Pin = GPIO_PIN_8,
		.IRQChannel = BSP_INT_ID_EXTI9_5,
		.isr = &EXTI9_5_IRQHandler,
		.irqType = EXTI9_5_IRQn,
	},
	{
		.GPIO_Pin = GPIO_PIN_9,
		.IRQChannel = BSP_INT_ID_EXTI9_5,
		.isr = &EXTI9_5_IRQHandler,
		.irqType = EXTI9_5_IRQn,
	},
	{
		.GPIO_Pin = GPIO_PIN_10,
		.IRQChannel = BSP_INT_ID_EXTI15_10,
		.isr = &EXTI15_10_IRQHandler,
		.irqType = EXTI15_10_IRQn,
	},
	{
		.GPIO_Pin = GPIO_PIN_11,
		.IRQChannel = BSP_INT_ID_EXTI15_10,
		.isr = &EXTI15_10_IRQHandler,
		.irqType = EXTI15_10_IRQn,
	},
	{
		.GPIO_Pin = GPIO_PIN_12,
		.IRQChannel = BSP_INT_ID_EXTI15_10,
		.isr = &EXTI15_10_IRQHandler,
		.irqType = EXTI15_10_IRQn,
	},
	{
		.GPIO_Pin = DEF_BIT_13,
		.IRQChannel = BSP_INT_ID_EXTI15_10,
		.isr = &EXTI15_10_IRQHandler,
		.irqType = EXTI15_10_IRQn,
	},
	{
		.GPIO_Pin = GPIO_PIN_14,
		.IRQChannel = BSP_INT_ID_EXTI15_10,
		.isr = &EXTI15_10_IRQHandler,
		.irqType = EXTI15_10_IRQn,
	},
	{
		.GPIO_Pin = GPIO_PIN_15,
		.IRQChannel = BSP_INT_ID_EXTI15_10,
		.isr = &EXTI15_10_IRQHandler,
		.irqType = EXTI15_10_IRQn,
	},
#endif
};
struct gpio_bank_ref g_gpio_bank_ref[] = {
#if defined(STDPERIPH_DRIVER)
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
#elif defined(STM32CUBEF4)
	{
		.GPIOx = GPIOA,
	},
		{
		.GPIOx = GPIOB,
	},
		{
		.GPIOx = GPIOC,
	},
		{
		.GPIOx = GPIOD,
	},
		{
		.GPIOx = GPIOE,
	},
		{
		.GPIOx = GPIOF,
	},
		{
		.GPIOx = GPIOG,
	},
		{
		.GPIOx = GPIOH,
	},
		{
		.GPIOx = GPIOI,
	},
#endif
};
#include <debug.h>
int 	gpio_open	(struct platform_device *dev, int flags){
	int ret = -EPERM;
	int bank = gpio_get_bank_index(dev->id);
	int pin = gpio_get_pin_index(dev->id);
	GPIO_InitTypeDef GPIO_InitStructure;

	struct gpio_platform_data* data = (struct gpio_platform_data*)dev->dev.platform_data;
	
	if(bank < 0 || bank >= GPIO_BANK_COUNT ||
		pin < 0 || pin >= GPIO_PIN_COUNT)
		return ret;
	
#if defined(STDPERIPH_DRIVER)
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	RCC_AHB1PeriphClockCmd(g_gpio_bank_ref[bank].RCC_AHB1Periph_GPIOx, ENABLE);
	GPIO_InitStructure.GPIO_Mode 	= data->dir;
	GPIO_InitStructure.GPIO_Pin 	= (((uint16_t)0x01) << pin);
	GPIO_InitStructure.GPIO_PuPd 	= data->pull;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIO_InitStructure);
	if(data->intr.mode != GPIO_INTR_MODE_DISABLE){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

		SYSCFG_EXTILineConfig(g_gpio_bank_ref[bank].EXTI_PortSourceGPIOx, pin);
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
	GPIO_InitStructure.Mode 	= data->dir;
	GPIO_InitStructure.Pin 		= g_gpio_pin_ref[pin].GPIO_Pin;
	GPIO_InitStructure.Pull 	= data->pull;
	GPIO_InitStructure.Speed 	= GPIO_SPEED_HIGH;
	if(data->intr.mode != GPIO_INTR_MODE_DISABLE){
		GPIO_InitStructure.Mode |= data->intr.mode | data->intr.trigger;
		BSP_IntVectSet(g_gpio_pin_ref[pin].IRQChannel, g_gpio_pin_ref[pin].isr);
		HAL_NVIC_SetPriority(g_gpio_pin_ref[pin].irqType, 6, 0);
		HAL_NVIC_EnableIRQ(g_gpio_pin_ref[pin].irqType);
	}
	HAL_GPIO_Init(g_gpio_bank_ref[bank].GPIOx, &GPIO_InitStructure);
#endif
	
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
#if defined(STDPERIPH_DRIVER)
		*p = GPIO_ReadInputDataBit(g_gpio_bank_ref[bank].GPIOx, g_gpio_pin_ref[pin].GPIO_Pin);
#elif defined(STM32CUBEF4)
		*p = (HAL_GPIO_ReadPin(g_gpio_bank_ref[bank].GPIOx, g_gpio_pin_ref[pin].GPIO_Pin) == GPIO_PIN_SET);
#endif
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
#if defined(STDPERIPH_DRIVER)
		if(p[0] == 0) GPIO_ResetBits(g_gpio_bank_ref[bank].GPIOx, (((uint16_t)0x01)<< pin));
		else GPIO_SetBits(g_gpio_bank_ref[bank].GPIOx, (((uint16_t)0x01)<< pin));
#elif defined(STM32CUBEF4)
		HAL_GPIO_WritePin(g_gpio_bank_ref[bank].GPIOx, g_gpio_pin_ref[pin].GPIO_Pin, (p[0] == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
		ret = 0;
	}	
		
	return ret;
}
int		gpio_ioctl	(struct platform_device *dev, int request, unsigned int arguments){
	int ret = -EPERM;
    int bank = gpio_get_bank_index(dev->id);
	int pin = gpio_get_pin_index(dev->id);
    
    switch(request){
        case GPIO_IOC_TOGGLE:{
#if defined(STDPERIPH_DRIVER)
            GPIO_ToggleBits(g_gpio_bank_ref[bank].GPIOx, g_gpio_pin_ref[pin].GPIO_Pin);
#elif defined(STM32CUBEF4)
            HAL_GPIO_TogglePin(g_gpio_bank_ref[bank].GPIOx, g_gpio_pin_ref[pin].GPIO_Pin);
#endif
            ret = 0;
            break;
        }
    }
    
	return ret;
}
int		gpio_select(struct platform_device *dev, int *readfd, int *writefd, int *exceptfd, int timeout){
	int ret = -EPERM;
#if defined(OS_FREERTOS)
	EventBits_t uxBits;
#elif defined(OS_UCOS)
	OS_FLAGS uxBits;
	OS_ERR p_err;
#endif
	uint8_t u8data;
	int bank = gpio_get_bank_index(dev->id);
	int pin = gpio_get_pin_index(dev->id);
	struct gpio_platform_data* data = (struct gpio_platform_data*)dev->dev.platform_data;
	
	if(readfd) 		*readfd = 0;
	if(writefd) 	*writefd = 0;
	if(exceptfd) 	*exceptfd = 0;
	if(readfd){
#if defined(OS_FREERTOS)
		uxBits = xEventGroupWaitBits(g_gpio_driver_arch_data.event,
				((uint32_t)1) << pin,
				pdTRUE,
				pdFALSE,
				timeout);
#elif defined(OS_UCOS)
//		LREP("s %d[%d] timeout=%d\r\n", bank, pin, timeout);
		uxBits = OSFlagPend(&g_gpio_driver_arch_data.event,
				((uint32_t)1) << pin,
				timeout,
				OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME,
				0,
				&p_err);
//		LREP("s %d[%d]=%04X\r\n", bank, pin, uxBits);

#endif
		if((p_err == OS_ERR_NONE) && (uxBits & (((uint32_t)1) << pin))) {
			*readfd = 1;
			ret = 1;
		}
		else ret = 0;
	}	
	return ret;
}
void EXTI0_IRQHandler(void) {
#if defined(OS_FREERTOS)
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
#elif defined(OS_UCOS)
	OS_ERR os_err;
	LED_TOGGLE(RED);
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_0);
	OSFlagPost(&g_gpio_driver_arch_data.event,
			((uint32_t)1 << 0),
			OS_OPT_POST_FLAG_SET,
			&os_err);
#endif
}
void EXTI1_IRQHandler(void) {
#if defined(OS_FREERTOS)
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
#elif defined(OS_UCOS)
	OS_ERR os_err;
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);
	OSFlagPost(&g_gpio_driver_arch_data.event,
			((uint32_t)1 << 1),
			OS_OPT_POST_FLAG_SET,
			&os_err);
#endif
}
void EXTI2_IRQHandler(void) {
#if defined(OS_FREERTOS)
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
#elif defined(OS_UCOS)
	OS_ERR os_err;
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_2);
	OSFlagPost(&g_gpio_driver_arch_data.event,
			((uint32_t)1 << 2),
			OS_OPT_POST_FLAG_SET,
			&os_err);
#endif
}
void EXTI3_IRQHandler(void) {
#if defined(OS_FREERTOS)
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
#elif defined(OS_UCOS)
	OS_ERR os_err;
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_3);
	OSFlagPost(&g_gpio_driver_arch_data.event,
			((uint32_t)1 << 3),
			OS_OPT_POST_FLAG_SET,
			&os_err);
#endif
}
void EXTI4_IRQHandler(void) {
#if defined(OS_FREERTOS)
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
#elif defined(OS_UCOS)
	OS_ERR os_err;
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_4);
	OSFlagPost(&g_gpio_driver_arch_data.event,
			((uint32_t)1 << 4),
			OS_OPT_POST_FLAG_SET,
			&os_err);
#endif
}
void EXTI9_5_IRQHandler(void) {
#if defined(OS_FREERTOS)
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
#elif defined(OS_UCOS)
	OS_ERR os_err;
	uint32_t flags = 0;
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_5))
		flags |= (uint32_t)1 << 5;
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_6))
		flags |= (uint32_t)1 << 6;
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_7))
		flags |= (uint32_t)1 << 7;
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_8))
		flags |= (uint32_t)1 << 8;
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_9))
		flags |= (uint32_t)1 << 9;
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);
	OSFlagPost(&g_gpio_driver_arch_data.event,
			flags,
			OS_OPT_POST_FLAG_SET,
			&os_err);
#endif
}
void EXTI15_10_IRQHandler(void) {
#if defined(OS_FREERTOS)
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
#elif defined(OS_UCOS)
	OS_ERR os_err;
	uint32_t flags = 0;
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_10))
		flags |= (uint32_t)1 << 10;
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_11))
		flags |= (uint32_t)1 << 11;
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_12))
		flags |= (uint32_t)1 << 12;
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_13))
		flags |= (uint32_t)1 << 13;
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_14))
		flags |= (uint32_t)1 << 14;
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_15))
		flags |= (uint32_t)1 << 15;
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	OSFlagPost(&g_gpio_driver_arch_data.event,
			flags,
			OS_OPT_POST_FLAG_SET,
			&os_err);
#endif
}
// end of file
