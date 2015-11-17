#include <drv_api.h>
#include <drv_usart.h>

struct usart_platform_data g_usart_1_data = {

};
struct platform_device g_usart_1_device = {
	.dev_name = "usart-1",
	.name     = "usart-drv",
	.id 	  = 0,
	.dev = {
		.platform_data = &g_usart_1_data,
	},
	.next = 0,
};

int board_register_devices(){
	platform_device_register(&g_usart_1_device);
	return 0;
}
//end of file


