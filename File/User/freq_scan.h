#ifndef __FREQ_SCAN_H__
#define __FREQ_SCAN_H__


#include "hsq_math.h"


void FREQ_SCAN_init(void);
void FREQ_SCAN_step(float dt);
float FREQ_SCAN_get_input(void);
void FREQ_SCAN_set_output(float out);

#endif

