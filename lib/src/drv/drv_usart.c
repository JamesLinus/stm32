#include "../include/drv_api.h"

int 	usart_init		(void);

static struct platform_driver g_usart_driver = {
	.driver		= {
		.name	= "usart-drv",
		.devices = 0,
	},
	.probe		= 0,
	.remove		= 0,
	.resume 	= 0,
	.suspend	= 0,
	.shutdown	= 0,

	.open 		= 0,
	.close 		= 0,
	.read 		= 0,
	.write 		= 0,
	.ioctl 		= 0,

	.next 		= 0,
};
int usart_init		(void){

	platform_driver_register(&g_usart_driver);
	return 0;
}
module_init(usart_init);
