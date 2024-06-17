#ifndef FOC_H
#define FOC_H

#include "stm32f10x.h"
#include "AS5600.h"
#include "Pwm.h"


#define PI 3.1415926
#define _3PI_2 4.71238898038f
#define _PI_3 1.0471975512
#define _PI_2 1.57079632679

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


extern float sensor_direction;
extern float pole_pairs;
extern float voltage_power_supply;
extern float zero_electric_angle;


void setPhaseVoltage(float Uq, float Ud, float angle_el);
float _normalizeAngle(float angle);
float _electricalAngle();



#endif 

