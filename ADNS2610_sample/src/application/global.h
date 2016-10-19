/*
 * global.h
 *
 *  Created on: 14.04.2011
 *      Author: mm
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_rtc.h"

#include "hw_config.h"
#include "USART_io.h"
#include "ADNS2610.h"

extern volatile uint32_t _sysTicks;
void delayMs(uint32_t msec);
void delayUs(uint32_t usec);

// PA5 - ADNS2610(SCK), PB11(5V) - ADNS2610(SDIO)


#endif /* GLOBAL_H_ */
