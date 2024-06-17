#ifndef Motor_H
#define Motor_H

#include "stm32f10x.h"
#include "FOC.h"
#include "Pid.h"
#include "Adc.h"


//初始化
void Motor_init();
//开环速度控制
float velocityOpenloop(float target_velocity);
//闭环位置控制(单闭环无电流环)
void PID_Pos_Set(float P, float I, float D, float limit);// PID_Pos_Set(0.10,0.01,0,6);
void PositionCloseloop(float position);//位置
//速度闭环(单闭环无电流环)
void PID_Vel_Set(float P, float I, float D, float limit);// PID_Pos_Set(0.001,0.01,0,6);
void VelocityCloseloop(float Velocity);
//电流环
void PID_Cur_Set(float P, float I, float D, float limit);//	PID_Cur_Set(10,5,0,6);
void CurrentCloseloop(float current);
//位置速度环
void Position_VelocityCloseloop(float position);// PID_Pos_Set(0.10,0.01,0,6);// PID_Pos_Set(0.10,1.5,0,6);
//速度电流环
void Velocity_CurrentCloseloop(float velocity);
//位置电流环
void Position_CurrentCloseloop(float position);



extern PID PID_Pos;
extern PID PID_Vel;
extern PID PID_Cur;
extern float position;
extern float velocity;
extern float current;



#endif 
