#include <string.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include <stm32f10x_spi.h>

#include "global.h"

#ifdef ENABLE_ADNS2610_SRC


/* Write operations, where data is going from the microcontroller to the ADNS-2610,
 * is always initiated by the microcontroller and consists of two bytes.
 * The first byte contains the address (seven bits) and has a “1” as its MSB
 * to indicate data direction.
 * The second byte contains the data. The transfer is synchronized by SCK.
 * The microcontroller changes SDIO on falling edges of SCK.
 * The ADNS-2610 reads SDIO on rising edges of SCK.
 */

//-----------------------------------------------------------------------------------------
static volatile uint8_t imageBuffer[18 * 18];

#define ADNS2610_CLK_PORT GPIOA
#define ADNS2610_CLK_PIN GPIO_Pin_5

#define ADNS2610_SDIO_PORT GPIOB
#define ADNS2610_SDIO_PIN GPIO_Pin_11

#define ADNS2610_SDIO_CR_REGISTER (ADNS2610_SDIO_PORT->CRH)
#define ADNS2610_SDIO_MASK (0xF << (3*4))
#define ADNS2610_SDIO_MASK_IN (0x4 << (3*4)) //  '01' Floating input (reset state)      '00' Input mode
#define ADNS2610_SDIO_MASK_OUT (0x3 << (3*4)) // '00' General purpose output push-pull, '11' 50MHz

// STM32F103 SPI can't be used for ADNS2610:
//		a. (MSTR=1, BIDIMODE=1, BIDIOE=0 can't stop SCK in time)
//		b. PA7 is not 5V FT
void ADNS2610_init() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = ADNS2610_CLK_PIN;
	GPIO_Init(ADNS2610_CLK_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ADNS2610_SDIO_PIN;
	GPIO_Init(ADNS2610_SDIO_PORT, &GPIO_InitStructure);
	ADNS2610_CLK_PORT->BSRR = ADNS2610_CLK_PIN; // = 1

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
}

//=========================================================================================
void ADNS2610_delay() {
	ADNS2610_delayUs(110); // >= 100us
}

void ADNS2610_delayUs(int16_t delay) {
	TIM_Cmd(TIM2, ENABLE);
	TIM_SetCounter(TIM2, 0);
	while(TIM_GetCounter(TIM2) < delay);
	TIM_Cmd(TIM2, DISABLE);
}

static void CLK_halfDelay() { // >= 2.5 us 4Mhz
	 uint32_t volatile tmp = 100;
	 while( tmp-- ) ;
}

static inline void sendByte(uint8_t b) {
	uint8_t msk  = 0x80;
	do {
		ADNS2610_CLK_PORT->BRR = ADNS2610_CLK_PIN; // = 0
		if(b & msk) {
			ADNS2610_SDIO_PORT->BSRR = ADNS2610_SDIO_PIN; // = 1
		} else {
			ADNS2610_SDIO_PORT->BRR = ADNS2610_SDIO_PIN; // = 0
		}
		CLK_halfDelay();
		ADNS2610_CLK_PORT->BSRR = ADNS2610_CLK_PIN; // = 1
		CLK_halfDelay();
		msk = msk >> 1;
	} while(msk != 0);
}

static inline uint8_t getByte() {
	uint8_t msk  = 0x80;
	uint8_t b = 0;
	do {
		ADNS2610_CLK_PORT->BRR = ADNS2610_CLK_PIN; // = 0
		CLK_halfDelay();
		if(ADNS2610_SDIO_PORT->IDR & ADNS2610_SDIO_PIN) {
			b |= msk;
		}
		ADNS2610_CLK_PORT->BSRR = ADNS2610_CLK_PIN; // = 1
		CLK_halfDelay();
		msk = msk >> 1;
	} while(msk != 0);
	return b;
}

void ANDS2610_setRegister(uint8_t regAddress, uint8_t data) {
	sendByte(regAddress | 0x80);
	sendByte(data);
	// There are minimum timing requirements between read and write commands on the serial port.
	ADNS2610_delay(); // delay for next operation
}

uint8_t ANDS2610_getRegister(uint8_t regAddress) {
	sendByte(regAddress);
	ADNS2610_SDIO_CR_REGISTER = (ADNS2610_SDIO_CR_REGISTER & (~ADNS2610_SDIO_MASK)) | ADNS2610_SDIO_MASK_IN;
	ADNS2610_delay();
	uint8_t b = getByte();
	ADNS2610_SDIO_CR_REGISTER = (ADNS2610_SDIO_CR_REGISTER & (~ADNS2610_SDIO_MASK)) | ADNS2610_SDIO_MASK_OUT;
	return b;
}

// first reset should be done after "Serial Port Transaction Timer = 90ms" delay from SMT32 start!
void ADNS2610_setAwake() {
	// "Configuration" address = 0;
	// data: 0x01 - Always awake
	ANDS2610_setRegister(0x00, 0x01);
}

// Time per 18x18 image scan:
//		0.214s 1008..1012 lostCnt(ADNS2610_delay = 110us, CLK_halfDelay = 8)
//		0.214s 691..701 lostCnt  (ADNS2610_delay = 100us, CLK_halfDelay = 20)
//		0.214s 517..520 lostCnt  (ADNS2610_delay = 100us, CLK_halfDelay = 30)
//		0.214s 377..380 lostCnt  (ADNS2610_delay = 150us, CLK_halfDelay = 30)
//		0.214s 52..54 lostCnt    (ADNS2610_delay = 100us, CLK_halfDelay = 100)
//		0.214s 39..41 lostCnt    (ADNS2610_delay = 120us, CLK_halfDelay = 100)
//		0.218s  0..1  lostCnt    (ADNS2610_delay = 200us, CLK_halfDelay = 100)
// Time per 18x1 image scan: 0.012 sec
uint8_t *ADNS2610_getImage(uint32_t sz, int *lost) {
 if(sz > sizeof(imageBuffer)) sz = sizeof(imageBuffer);
 ANDS2610_setRegister(0x08, 0x2A);
 uint32_t n = 0;
 int lostCnt = 0;
 for(int i = 0; i < 10000; i++) {
	 uint8_t b = ANDS2610_getRegister(0x08);
	 if((b & 0x40) == 0) {
		 lostCnt++;
		 continue;
	 }
	 imageBuffer[n++] = b & 0xBF;
	 if(n == sz) {
		 *lost = lostCnt;
		 return (uint8_t*)(&imageBuffer[0]);
	 }
 }
 return NULL;
}


///* sample...
void ADNS2610_test() {
	delayMs(T_SPTT);
	ADNS2610_init();
	delayMs(T_SPTT);
	ADNS2610_setAwake();
	uint8_t reg01 = ANDS2610_getRegister(0x01);
	uint8_t reg11 = ANDS2610_getRegister(0x11);
	USART_DBG_printf("ADNS2610 registers: [0x01]=%x [0x11]=%x\n", reg01, reg11);
	if((reg01 & 0xE1) != 0x01 || (reg11 & 0x0F) != 0x0F) {
	 USART_DBG_printf("Critical error. Wrong ADNS2610 registers values");
	}
	uint32_t t1 = _sysTicks;
	int lost;
	uint8_t *img = ADNS2610_getImage(18, &lost);
	if(img == NULL) {
		USART_DBG_puts("Get image from ADNS2610 register [0x08] timeout!!!\n");
	}
	uint32_t dt = _sysTicks-t1;
	USART_DBG_printf("ADNS2610 get 18 pixel image time:%d ms, [0x08] image bytes without Data_Valid bit:%d\n", dt, lost);

	uint16_t sz = 18*18;
	while(1) {
		int lost;
		uint8_t *img = ADNS2610_getImage(sz, &lost);
	//		uint32_t dt = _sysTicks-t1;
	//		USART_DBG_printf("DT:%d %d\n", dt, lost);
		if(img == NULL) USART_DBG_puts("Get image from ADNS2610 register [0x08] timeout!!!\n");
		else USART_DBG_bin(img, sz);
	//		delayMs(100);
	//		for(int i = 0; i < 18; i++) {USART_DBG_hexDump(img+i*18, 18);	USART_DBG_puts("\n");	}
	//		USART_DBG_puts("-----------\n");
   }
}
//*/

#endif // ENABLE_ADNS2610_SRC

