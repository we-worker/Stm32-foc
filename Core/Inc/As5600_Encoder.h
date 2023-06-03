#ifndef __AS5600_H
#define __AS5600_H

#include "main.h"


#define _raw_ang_hi  0x0c
#define _raw_ang_lo  0x0d
#define _ams5600_Address (0x36<<1)

float get_angle(void);
float get_angle_total(void);

#endif