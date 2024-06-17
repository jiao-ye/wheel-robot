#ifndef LOWPASS_H
#define LOWPASS_H


#include "stm32f10x.h"
#include "FOC.h"
#include "Usart.h"

typedef struct 
{
	float Tf; 
	float y_prev; 
	unsigned long timestamp_prev;  
} LowPassFilter;


extern LowPassFilter  LPF_current,LPF_velocity;

void LOWPass_Init();
float LowPassFilter_operator(LowPassFilter *Lfi,float x);


#endif