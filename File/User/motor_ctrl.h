#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__
#include "hsq_math.h"

void MOTOR_CTRL_init(void);
void MOTRO_CTRL_step(float dt);
void MOTOR_SIMULATE_step(float dt);
void MOTOR_SCAN_step(float dt);

#endif

