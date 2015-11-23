/*
 * drv_usart.h
 *
 *  Created on: Nov 17, 2015
 *      Author: dev
 */

#ifndef INCLUDE_DRV_USART_H_
#define INCLUDE_DRV_USART_H_

#define USART_MODULE_COUNT		(6)

struct usart_platform_data{
	unsigned char 	tx_pin;	/*bank * 16 + pin*/
	unsigned char	rx_pin;	/*bank * 16 + pin*/

	void*	__drv_usart_base;
	int		__drv_state;
};




#endif /* INCLUDE_DRV_USART_H_ */
