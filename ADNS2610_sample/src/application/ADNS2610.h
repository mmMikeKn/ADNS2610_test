#ifndef ADNS2610_H_
#define ADNS2610_H_


#define ENABLE_ADNS2610_SRC

#ifdef ENABLE_ADNS2610_SRC
void ADNS2610_init();
//Serial Port Transaction Timer. Serial port will reset if current transaction is not complete within tSPTT
// T_SPTT > 90ms
#define T_SPTT 150

// ============== this functions can be used only if (doInBackground == FALSE) ======
// first reset should be done after "Serial Port Transaction Timer = 90ms" delay from SMT32 start!
void ADNS2610_setAwake();
void ADNS2610_delayUs(int16_t delay); // TIM2 for timeout
uint8_t ANDS2610_getRegister(uint8_t regAddress);
void ANDS2610_setRegister(uint8_t regAddress, uint8_t data);
uint8_t *ADNS2610_getImage(uint32_t sz, int *lost);
// ==================================================================================

#endif //ENABLE_ADNS2610_SRC
#endif /* ADNS2610_H_ */
