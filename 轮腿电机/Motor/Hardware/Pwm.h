#ifndef PWM_H
#define PWM_H

#include "stm32f10x.h"

#define Motor_Enable GPIO_SetBits(GPIOB,GPIO_Pin_15);
#define Motor_disable GPIO_ResetBits(GPIOB,GPIO_Pin_15);

void Pwm_Init();
void GPIO_En_init();
void Set_Pwm(uint8_t channel, uint16_t value);

#endif 
