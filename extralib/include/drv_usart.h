/*
 * drv_usart.h
 *
 *  Created on: Nov 17, 2015
 *      Author: dev
 */

#ifndef INCLUDE_DRV_USART_H_
#define INCLUDE_DRV_USART_H_

#define USART_MODULE_COUNT		(6)
#if defined(STM32CUBEF4)
#include <stm32f4xx_hal_uart.h>
#endif

struct usart_platform_data{
	unsigned char 	tx_pin;	/*bank * 16 + pin*/
	unsigned char	rx_pin;	/*bank * 16 + pin*/
#if defined(STDPERIPH_DRIVER)
	void*	__drv_usart_base;
#elif defined(STM32CUBEF4)
	UART_HandleTypeDef	__drv_usart_base;
#endif
	int		__drv_state;
};




#endif /* INCLUDE_DRV_USART_H_ */
