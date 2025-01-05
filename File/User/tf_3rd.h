#ifndef __TF_3RD_H__
#define __TF_3RD_H__
#include "hsq_math.h"

typedef struct 
{
    float dt; // 采样周期 s
    float num[4]; // G(z)分子系数
    float den[4]; // G(z)分母系数
    float r; // 输入信号 r(k)
    float u; // 输出信号 u(k)
    float r_pre; // r(k-1)
    float r_pre2; // r(k-2)
    float r_pre3; // r(k-3)
    float u_pre;  // u(k-1)
    float u_pre2; // u(k-2)
    float u_pre3; // u(k-3)
}TF_3RD_t, *TF_3RD_h;

void TF_3RD_init(TF_3RD_h h, float num_0, float num_1, float num_2, float num_3, float den_0, float den_1, float den_2, float den_3);
float TF_3RD_step(TF_3RD_h h, float dt, float r);
float TF_3RD_get_out(TF_3RD_h h);

#endif

