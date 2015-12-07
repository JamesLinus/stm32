#ifndef DEBUG_H__
#define DEBUG_H__
#include <drv_gpio.h>
enum LED{
	LED_GREEN = 0,
	LED_RED,
	LED_BLUE,
	LED_ORANGE
};
void LED_ON(int index);
void LED_OFF(int index);
void LED_TOGGLE(int index);

//void LREP(char* s, ...);
//#define LREP_WARN(s, args...) LREP("%d@%s " s, __LINE__, __FILE__, ##args)

extern int g_fd_led[];
#define LED_ON(led) {uint8_t val = 1; write(g_fd_led[LED_##led], &val, 1);}
#define LED_OFF(led) {uint8_t val = 0; write(g_fd_led[LED_##led], &val, 1);}
#define LED_TOGGLE(led) {ioctl(g_fd_led[LED_##led], GPIO_IOCTL_TOGGLE, 0);}
#endif
