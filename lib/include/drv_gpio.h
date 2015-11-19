#ifndef DRV_GPIO_H__
#define DRV_GPIO_H__
#include <stdint.h>
#include "stm32f4xx_gpio.h"

#define GPIO_OUTPUT	GPIO_Mode_OUT
#define GPIO_INPUT	GPIO_Mode_IN
#define GPIO_NOPULL GPIO_PuPd_NOPULL
struct gpio_platform_data {
	uint8_t dir;
	uint8_t	pull;
};
#define GPIO_BANK_COUNT		9
#define GPIO_PIN_COUNT		16
#endif
