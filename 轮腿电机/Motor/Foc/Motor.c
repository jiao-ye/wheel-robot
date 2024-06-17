#include "Motor.h"


float shaft_angle=0,open_loop_timestamp=0;
float position,velocity,current;

void Motor_init()
{
    float angle;

    for(int i=0; i<=500; i++)
	{
		angle = _3PI_2 + _2PI * i / 500.0;
		setPhaseVoltage(2.5, 0,  angle);
		Delay_ms(2);
	}
	
	for(int i=500; i>=0; i--) 
	{
		angle = _3PI_2 + _2PI * i / 500.0 ;
		setPhaseVoltage(2.5, 0,  angle);
		Delay_ms(2);
	}
	setPhaseVoltage(0, 0,  angle);
	Delay_ms(100);
	setPhaseVoltage(3, 0,_3PI_2);
	Delay_ms(300);
	zero_electric_angle=_electricalAngle();
  	setPhaseVoltage(0, 0,_3PI_2);

}



//开环速度控制
float velocityOpenloop(float target_velocity)
{
	unsigned long now_us;
	float Ts,Uq;
	
	now_us = SysTick->VAL; //_micros();
	if(now_us<open_loop_timestamp)Ts = (float)(open_loop_timestamp - now_us)/9*1e-6;
	else
		Ts = (float)(0xFFFFFF - now_us + open_loop_timestamp)/9*1e-6;
	open_loop_timestamp=now_us;  //save timestamp for next call
  // quick fix for strange cases (micros overflow)
  if(Ts == 0 || Ts > 0.5) Ts = 1e-3; 
	
	// calculate the necessary angle to achieve target velocity
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts); 
	
	Uq = 3;
	// set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Uq,  0, shaft_angle*pole_pairs);
	
	return Uq;
	


}

//位置闭环
PID PID_Pos;

void PID_Pos_Set(float P, float I, float D, float limit)
{
    PID_Pos.P = P;
    PID_Pos.I = I;
    PID_Pos.D = D;
    PID_Pos.limit = limit;
}

void PositionCloseloop(float position)
{
    float output;
    output=PID_operator(&PID_Pos,(position-sensor_direction*Get_Angel())*180/PI);
    setPhaseVoltage(output,0,_electricalAngle());
}

//速度闭环
PID PID_Vel;

void PID_Vel_Set(float P, float I, float D, float limit)
{
    PID_Vel.P = P;
    PID_Vel.I = I;
    PID_Vel.D = D;
    PID_Vel.limit = limit;
}

void VelocityCloseloop(float Velocity)
{
    float output;
    output=PID_operator(&PID_Vel,(Velocity-sensor_direction*Get_Velocity())*180/PI);
    setPhaseVoltage(output,0,_electricalAngle());
    
}

//电流环
PID PID_Cur;
void PID_Cur_Set(float P, float I, float D, float limit)
{
    PID_Cur.P = P;
    PID_Cur.I = I;
    PID_Cur.D = D;
    PID_Cur.limit = limit;
}

void CurrentCloseloop(float current)
{
    float output;
    output=PID_operator(&PID_Cur,(current-Get_Current()));
    setPhaseVoltage(output,0,_electricalAngle());
	
}




//位置速度闭环

void Position_VelocityCloseloop(float position)
{
    float output;
    output=PID_operator(&PID_Pos,(position-sensor_direction*Get_Angel())*180/PI);
	VelocityCloseloop(output);
}

//速度电流闭环
void Velocity_CurrentCloseloop(float velocity)
{
    float output;
    output=PID_operator(&PID_Vel,(velocity-sensor_direction*Get_Velocity())*180/PI);
	CurrentCloseloop(output);
}

//位置电流闭环
void Position_CurrentCloseloop(float position)
{
    float output;
    output=PID_operator(&PID_Pos,(position-sensor_direction*Get_Angel())*180/PI);
    CurrentCloseloop(output);
}