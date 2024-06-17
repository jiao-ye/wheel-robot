#include "Pwm.h"

void Pwm_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef         TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	
	TIM_TimeBaseInitStructure.TIM_Prescaler=1-1;
	TIM_TimeBaseInitStructure.TIM_Period=1440-1;  
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=0;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	TIM_OC3Init(TIM1,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM1,ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM1,ENABLE);
	
}

void Set_Pwm(uint8_t channel, uint16_t value)
{
	switch (channel) 
	{
		case 1 : TIM_SetCompare1(TIM1,value);break;
		case 2 : TIM_SetCompare2(TIM1,value);break;
		case 3 : TIM_SetCompare3(TIM1,value);break;
	}
}


void TIM4_1ms_Init(void)  
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	
	TIM_TimeBaseInitStructure.TIM_Period = 1000-1;      //1ms
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;       //72分频=1MHz
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM4,ENABLE);
}

void GPIO_En_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);//使能GPIOA,GPIOB,GPIOC,AFIO;
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;          //PB9是motor1的使能
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	Motor_Enable;
}