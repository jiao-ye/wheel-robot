#ifndef AS5600_H
#define AS5600_H

#include "stm32f10x.h"
#include "Delay.h"
#include "math.h"
#include "FOC.h"
#include "Lowpass.h"

#define AS5600 0x36
#define AS5600_ADDRESS (AS5600 <<1 )
#define AS5600_READ_ADDRESS (AS5600 << 1 | 1)
#define AS5600_WRITE_ADDRESS (AS5600 << 1 | 0 )

 
#define AS5600_RAW_ANGLE_REGISTER_HIGH  0x0E //数据寄存器
#define AS5600_RAW_ANGLE_REGISTER_LOW  0x0F //数据寄存器

#define _2PI 6.28318530718f
#define PI 3.1415926

typedef struct 
{
	long  velocity_calc_timestamp;  //速度计时，用于计算速度
	long  cpr;                      //编码器分辨率，AS5600=12bit(4096)
	long  angle_data_prev;          //获取角度用
	float angle_prev;               //获取速度用
	float full_rotation_offset;     //角度累加
} ENCODER_TypeDef;



void As5600_Init();
float Get_Angel();
float Get_Velocity();
float Get_Angel_Notrack();
float Get_Velocity2();
#endif 
