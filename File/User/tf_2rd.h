/*
 * @Author: hsq007 2267222816@qq.com
 * @Date: 2024-12-15 20:50:28
 * @LastEditors: hsq007 2267222816@qq.com
 * @LastEditTime: 2024-12-15 22:27:30
 * @FilePath: \20241214-M2006电机实验\File\User\tf_2rd.h
 * @Description: 有两个延迟的脉冲传递函数实现
 */
#ifndef __TF_2RD_H__
#define __TF_2RD_H__
#include "hsq_math.h"

typedef struct 
{
    float dt; // 采样周期 s
    float num[3]; // G(z)分子系数
    float den[3]; // G(z)分母系数
    float r; // 输入信号 r(k)
    float u; // 输出信号 u(k)
    float r_pre; // r(k-1)
    float r_pre2; // r(k-2)
    float u_pre;  // u(k-1)
    float u_pre2; // u(k-2)
}TF_2RD_t, *TF_2RD_h;

void TF_2RD_init(TF_2RD_h h, float num_0, float num_1, float num_2, float den_0, float den_1, float den_2);
float TF_2RD_step(TF_2RD_h h, float dt, float r);
float TF_2RD_get_out(TF_2RD_h h);

#endif



