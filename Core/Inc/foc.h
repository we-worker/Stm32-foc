#ifndef __FOC_H
#define __FOC_H

#include "main.h"
#include "math.h"

void Motoer_test(void);
void svpwm_test(void);

void foc2_test(void);
float velocityOpenloop(float target_velocity);

#endif