#ifndef DEBUG_H__
#define DEBUG_H__
enum LED{
	LED_GREEN = 0,
	LED_RED,
	LED_BLUE,
	LED_ORANGE
};
void LED_ON(int index);
void LED_OFF(int index);
void LED_TOGGLE(int index);

void LREP(char* s, ...);
#define LREP_WARN(s, args...) LREP("%d@%s " s, __LINE__, __FILE__, ##args)
#endif
