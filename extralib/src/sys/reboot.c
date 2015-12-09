#include <reboot.h>
#include "stm32f4xx.h"
void reboot(){
	NVIC_SystemReset();
}
// end of file
