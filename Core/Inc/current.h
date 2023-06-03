#ifndef CURRENT_H
#define CURRENT_H

#include "main.h"

extern ADC_HandleTypeDef hadc1;

typedef struct
{
     uint32_t	ADC_Value[2];
		 uint16_t i_a,i_b; //ºÁ°²
		 uint32_t iAlpha,iBeta;
} IPhase_s;

#define IPhase_default {0,0,0,0,0,0}


void ADC_READ(void);

#endif // CURRENT_H