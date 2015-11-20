#ifndef DRV_GPIO_H__
#define DRV_GPIO_H__
#include <stdint.h>
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "misc.h"

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
#define GPIO_PIN_INVALID	0xff

// pin = bank_index * 32 + pin_index
// bank_id:
//   A-0, B-1, C-2, D-3, E-4, F-5, G-6, H-7, I-8
#define gpio_get_bank_index(pin_id) (pin_id / GPIO_PIN_COUNT)
#define gpio_get_pin_index(pin_id)  (pin_id % GPIO_PIN_COUNT)
#define gpio_get_pin(bank, pin)		((bank - 'A') * 16 + pin) // bank = 'A', 'B', ...

struct gpio_bank_ref {
	GPIO_TypeDef* 	GPIOx;
	uint32_t 		RCC_AHB1Periph_GPIOx;
	uint32_t		EXTI_PortSourceGPIOx;
};
struct gpio_pin_ref {
	uint8_t NVIC_IRQChannel;
	uint16_t GPIO_PinSource;
	uint16_t GPIO_Pin;
};
extern struct gpio_bank_ref g_gpio_bank_ref[];
extern struct gpio_pin_ref  g_gpio_pin_ref[];
#endif
