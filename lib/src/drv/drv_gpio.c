#include "../include/drv_api.h"
#include "../include/drv_gpio.h"

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

int 	gpio_init		(void);
int 	gpio_open		(struct platform_device *dev, int flags);
int 	gpio_close		(struct platform_device *dev);
int		gpio_write		(struct platform_device *dev, const void* buf, int count);
int		gpio_read		(struct platform_device *dev, void* buf, int count);
int		gpio_ioctl		(struct platform_device *dev, int request, unsigned int arguments);
int		gpio_select	(struct platform_device *device, int *readfd, int *writefd, int *exceptfd, int timeout);

struct gpio_driver_arch_data{
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

int gpio_init		(void){;
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
};
struct gpio_param_ref g_gpio_param_ref[] = {
	{
		.GPIOx = GPIOA,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOA
	},
		{
		.GPIOx = GPIOB,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOB
	},
		{
		.GPIOx = GPIOC,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOC
	},
		{
		.GPIOx = GPIOD,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOD
	},
		{
		.GPIOx = GPIOE,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOE
	},
		{
		.GPIOx = GPIOF,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOF
	},
		{
		.GPIOx = GPIOG,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOG
	},
		{
		.GPIOx = GPIOH,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOH
	},
		{
		.GPIOx = GPIOI,
		.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOI
	},
};
//extern void LREP(char* s, ...);
int 	gpio_open		(struct platform_device *dev, int flags){
	int ret = -EPERM;
	int bank = gpio_get_bank_index(dev->id);
	int pin = gpio_get_pin_index(dev->id);
	GPIO_InitTypeDef GPIO_InitStructure;
	struct gpio_platform_data* data = (struct gpio_platform_data*)dev->dev.platform_data;
	
	if(bank < 0 || bank >= GPIO_BANK_COUNT ||
		pin < 0 || pin >= GPIO_PIN_COUNT)
		return ret;
	
	RCC_AHB1PeriphClockCmd(g_gpio_param_ref[bank].RCC_AHB1Periph_GPIOx, ENABLE);
	GPIO_InitStructure.GPIO_Mode 	= data->dir;
	GPIO_InitStructure.GPIO_Pin 	= (((uint16_t)0x01) << pin);
	GPIO_InitStructure.GPIO_PuPd 	= data->pull;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(g_gpio_param_ref[bank].GPIOx, &GPIO_InitStructure);
	ret = 0;	
	return ret;
}
int 	gpio_close		(struct platform_device *dev){
	int ret = -EPERM;
	return ret;
}
int		gpio_read		(struct platform_device *dev, void* buf, int count){
	int ret = -EPERM;
	struct gpio_platform_data* data = (struct gpio_platform_data*)dev->dev.platform_data;
	unsigned char* p = (unsigned char*)buf;

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
	
	return ret;
}
// end of file
