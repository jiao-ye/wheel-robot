#ifndef __ADC_H
#define __ADC_H	

#include "stm32f10x.h"
#include "Delay.h"
#include "math.h"
#include "Lowpass.h"
#include "Usart.h"
#define	_1_SQRT3 0.57735026919
#define _2_SQRT3 1.15470053838


void ADC_Init_(void);
uint16_t analogRead(uint8_t ch);
float _readADCVoltage(uint8_t ch);
void CurrSense_init();
float Get_Current();

void CurrSense_getPhaseCurrents();
#endif 
