#ifndef __Deng_FOC_H
#define __Deng_FOC_H

#include "main.h"
#include "math.h"

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//宏定义实现的一个约束函数,用于限制一个值的范围。

void Motoer_test(void);
void svpwm_test(void);

void foc2_test(void);
float velocityOpenloop(float target_velocity);
void Location_closed_loop(void);

extern uint16_t    ADC_Value[2];

#endif