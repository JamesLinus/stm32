#include <drv_api.h>
#include <drv_usart.h>
#include <drv_gpio.h>
/*usart*/
struct usart_platform_data g_usart_1_data;
struct platform_device g_usart_1_device = {
	.dev_name = "lrep",
	.name     = "usart-drv",
	.id 	  = 0,
	.dev = {
		.platform_data = &g_usart_1_data,
	},
	.next = 0,
};
/*gpio*/
struct gpio_platform_data g_gpio_data = {
	.dir = GPIO_OUTPUT,
	.pull = GPIO_NOPULL,
};
struct platform_device g_gpio_led_red_device = {
	.dev_name = "led-red",
	.name     = "gpio-drv",
	.dev = {
		.platform_data = &g_gpio_data,
	},
	.id = (3 * 16 + 14),	// D14
	.next = 0,
};

int board_register_devices(){
	platform_device_register(&g_usart_1_device);
	platform_device_register(&g_gpio_led_red_device);
	return 0;
}
//end of file


