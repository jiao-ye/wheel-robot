#include "Lowpass.h"


LowPassFilter  LPF_current,LPF_velocity;

void LOWPass_Init()
{
    LPF_current.timestamp_prev= SysTick->VAL;
    LPF_current.Tf=0.05;

    LPF_velocity.timestamp_prev= SysTick->VAL;
    LPF_velocity.Tf=0.01;
}


float LowPassFilter_operator(LowPassFilter *Lfi,float x)
{

    unsigned long timestamp;
	float Ts;

	timestamp = SysTick->VAL; 
	if(timestamp<Lfi->timestamp_prev)Ts = (float)(Lfi->timestamp_prev - timestamp)/9*1e-6;
	else
		Ts = (float)(0xFFFFFF - timestamp + Lfi->timestamp_prev)/9*1e-6;

	if(Ts == 0 || Ts > 0.5) Ts = 1e-3;

    float alpha = Lfi->Tf/(Lfi->Tf + Ts);
    float y = alpha*Lfi->y_prev + (1.0f - alpha)*x;
    Lfi->y_prev = y;
    Lfi->timestamp_prev = timestamp;
    return y;
}

// ENCODER_TypeDef E4;

// float LowPassFilter_operator_current(float x,float Tf)
// {

//     unsigned long timestamp;
// 	float Ts;

// 	timestamp = SysTick->VAL; 
// 	if(timestamp<E4.velocity_calc_timestamp)Ts = (float)(E4.velocity_calc_timestamp - timestamp)/9*1e-6;
// 	else
// 		Ts = (float)(0xFFFFFF - timestamp + E4.velocity_calc_timestamp)/9*1e-6;

// 	if(Ts == 0 || Ts > 0.5) Ts = 1e-3;

//     float alpha = Tf/(Tf + Ts);
//     float y = alpha*y_prev + (1.0f - alpha)*x;
//     y_prev = y;
//     timestamp_prev = timestamp;
//     return y;
// }
