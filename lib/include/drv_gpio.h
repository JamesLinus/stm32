#ifndef DRV_GPIO_H__
#define DRV_GPIO_H__
#include <stdint.h>
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"

#define GPIO_OUTPUT	GPIO_Mode_OUT
#define GPIO_INPUT	GPIO_Mode_IN
#define GPIO_NOPULL GPIO_PuPd_NOPULL

#define GPIO_INTR_MODE_INTERRUPT	EXTI_Mode_Interrupt
#define GPIO_INTR_MODE_EVENT		EXTI_Mode_Event
#define GPIO_INTR_MODE_DISABLE		0xff

#define GPIO_INTR_TRIGGER_RISING			EXTI_Trigger_Rising
#define GPIO_INTR_TRIGGER_FALLING			EXTI_Trigger_Falling
#define GPIO_INTR_TRIGGER_RISING_FALLING	EXTI_Trigger_Rising_Falling
struct gpio_platform_data_interrupt{
	uint8_t mode;
	uint8_t trigger;
};
struct gpio_platform_data {
	uint8_t dir;
	uint8_t	pull;
	struct gpio_platform_data_interrupt intr;
};
#define GPIO_BANK_COUNT		9
#define GPIO_PIN_COUNT		16
#endif
