#ifndef __Deng_FOC_H
#define __Deng_FOC_H

#include "main.h"
#include "math.h"

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//�궨��ʵ�ֵ�һ��Լ������,��������һ��ֵ�ķ�Χ��

void Motoer_test(void);
void svpwm_test(void);

void foc2_test(void);
float velocityOpenloop(float target_velocity);
void Location_closed_loop(void);

extern uint16_t    ADC_Value[2];

#endif