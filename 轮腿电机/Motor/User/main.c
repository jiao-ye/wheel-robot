#include "stm32f10x.h"                  // Device header
#include "AS5600.h"
#include "Usart.h"
#include "Pwm.h"
#include "FOC.h"
#include "Motor.h" 
#include "Adc.h"

int count;

int main()
{
	GPIO_En_init();
	uart_init(115200);
	As5600_Init();
	printf("AS5600初始化\r\n");
	Pwm_Init();
	Motor_init();
	CurrSense_init();
	Delay_ms(1000);
	PID_Pos_Set(0.10,0.01,0,6);
	PID_Vel_Set(0.10,0.01,0,6);
	PID_Cur_Set(0.1,0.1,0,6);
	LOWPass_Init();
	while (1)
	{

		velocityOpenloop(20);

		printf("%.3f,%.2f,%.2f,%.2f\n",Get_Current(),PID_Cur.P,PID_Cur.I,PID_Cur.D);


		
	


	}
}
