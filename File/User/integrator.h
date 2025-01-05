/**
 * 积分模块
 * 
 * **/

#ifndef __INTEGRATOR_H__
#define __INTEGRATOR_H__

typedef struct
{
    float dt;
    float gain; // 增益
    float r; // 输入
    float u; // 输出
}INTEGRATOR_t, *INTEGRATOR_h;

void INTEGRATOR_init(INTEGRATOR_h h, float gain, float u_0);
float INTEGRATOR_step(INTEGRATOR_h h, float dt, float r);

#endif



