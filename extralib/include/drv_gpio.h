#ifndef DRV_GPIO_H__
#define DRV_GPIO_H__
#include <stdint.h>
#include <lib_defines.h>
#if defined(STDPERIPH_DRIVER)
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#define GPIO_OUTPUT							GPIO_Mode_OUT
#define GPIO_INPUT							GPIO_Mode_IN
#define GPIO_NOPULL 						GPIO_PuPd_NOPULL

#define GPIO_INTR_MODE_INTERRUPT			EXTI_Mode_Interrupt
#define GPIO_INTR_MODE_EVENT				EXTI_Mode_Event
#define GPIO_INTR_MODE_DISABLE				0xff

#define GPIO_INTR_TRIGGER_RISING			EXTI_Trigger_Rising
#define GPIO_INTR_TRIGGER_FALLING			EXTI_Trigger_Falling
#define GPIO_INTR_TRIGGER_RISING_FALLING	EXTI_Trigger_Rising_Falling
#elif defined(STM32CUBEF4)
#include <stm32f407xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#include <bsp.h>
#define GPIO_OUTPUT							GPIO_MODE_OUTPUT_PP
#define GPIO_INPUT							GPIO_MODE_INPUT
//#define GPIO_NOPULL 						GPIO_NOPULL

#define GPIO_INTR_MODE_INTERRUPT			((uint32_t)0x10010000)
#define GPIO_INTR_MODE_EVENT				((uint32_t)0x10020000)
#define GPIO_INTR_MODE_DISABLE				0

#define GPIO_INTR_TRIGGER_RISING			((uint32_t)0x00100000)
#define GPIO_INTR_TRIGGER_FALLING			((uint32_t)0x00200000)
#define GPIO_INTR_TRIGGER_RISING_FALLING	((uint32_t)0x00300000)
#endif

struct gpio_platform_data_interrupt{
	uint32_t mode;
	uint32_t trigger;
};
struct gpio_platform_data {
	uint32_t dir;
	uint32_t pull;
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
#if defined(STDPERIPH_DRIVER)
	uint32_t 		RCC_AHB1Periph_GPIOx;
	uint32_t		EXTI_PortSourceGPIOx;
#endif
};
struct gpio_pin_ref {
	uint16_t GPIO_Pin;
#if defined(STDPERIPH_DRIVER)
	uint8_t NVIC_IRQChannel;
	uint16_t GPIO_PinSource;
#elif defined(STM32CUBEF4)
	uint16_t IRQChannel;
	CPU_FNCT_VOID isr;
	IRQn_Type irqType;
#endif
};
extern struct gpio_bank_ref g_gpio_bank_ref[];
extern struct gpio_pin_ref  g_gpio_pin_ref[];

#define GPIO_IOC_TOGGLE   0x01
#endif
