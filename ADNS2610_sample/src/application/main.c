#include <stdlib.h>
#include <string.h>
#include "global.h"


void delayMs(uint32_t msec)  {
	uint32_t tmp = 7000 * msec  ;
	while( tmp-- ) __NOP();
}

inline void delayUs(uint32_t usec)  {
	uint32_t tmp = 7 * usec  ;
	while( tmp-- ) __NOP();
}

void ADNS2610_test();
int main() {
	SystemStartup();
	USART_init();
	ADNS2610_test();
}


