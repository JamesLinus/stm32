#include <drv_api.h>
#include <drv_usart.h>
#include <drv_gpio.h>
/*usart*/
struct usart_platform_data g_usart_lrep_data;
struct platform_device g_usart_lrep_device = {
	.dev_name 	= "lrep",
	.name     	= "usart-drv",
	.id 	  	= 0,
	.dev 		= {
		.platform_data = &g_usart_lrep_data,
	},
	.next 		= 0,
};
/*gpio*/
struct gpio_platform_data g_gpio_led_data = {
	.dir 	= GPIO_OUTPUT,
	.pull 	= GPIO_NOPULL,
};
struct gpio_platform_data g_gpio_button_data = {
	.dir 	= GPIO_INPUT,
	.pull 	= GPIO_NOPULL,
	.intr 	= {
		.mode 		= GPIO_INTR_MODE_INTERRUPT,
		.trigger 	= GPIO_INTR_TRIGGER_RISING_FALLING,
	},
};
struct platform_device g_gpio_led_green_device = {
	.dev_name 	= "led-green",
	.name     	= "gpio-drv",
	.dev 		= {
		.platform_data = &g_gpio_led_data,
	},
	.id 		= (3 * 16 + 12),	// D12
	.next 		= 0,
};
struct platform_device g_gpio_led_orange_device = {
	.dev_name 	= "led-orange",
	.name     	= "gpio-drv",
	.dev 		= {
		.platform_data = &g_gpio_led_data,
	},
	.id 		= (3 * 16 + 13),	// D13
	.next 		= 0,
};
struct platform_device g_gpio_led_red_device = {
	.dev_name 	= "led-red",
	.name     	= "gpio-drv",
	.dev 		= {
		.platform_data = &g_gpio_led_data,
	},
	.id			= (3 * 16 + 14),	// D14
	.next 		= 0,
};
struct platform_device g_gpio_led_blue_device = {
	.dev_name 	= "led-blue",
	.name     	= "gpio-drv",
	.dev 		= {
		.platform_data = &g_gpio_led_data,
	},
	.id 		= (3 * 16 + 15),	// D15
	.next 		= 0,
};
struct platform_device g_gpio_button_device = {
	.dev_name 	= "button",
	.name     	= "gpio-drv",
	.dev 		= {
		.platform_data = &g_gpio_button_data,
	},
	.id 		= (0 * 16 + 0),	// A0
	.next 		= 0,
};

int board_register_devices(){
	platform_device_register(&g_usart_lrep_device);
	
	platform_device_register(&g_gpio_led_red_device);
	platform_device_register(&g_gpio_led_green_device);
	platform_device_register(&g_gpio_led_orange_device);
	platform_device_register(&g_gpio_led_blue_device);
	platform_device_register(&g_gpio_button_device);
	return 0;
}
//end of file


