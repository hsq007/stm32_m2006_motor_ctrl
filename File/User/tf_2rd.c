/*
 * @Author: hsq007 2267222816@qq.com
 * @Date: 2024-12-15 20:50:21
 * @LastEditors: hsq007 2267222816@qq.com
 * @LastEditTime: 2024-12-15 21:02:39
 * @FilePath: \20241214-M2006电机实验\File\User\tf.c
 * @Description: 有两个延迟的脉冲传递函数实现
 */
#include "tf_2rd.h"

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

TF_2RD_t g_tf_2rd = {
    .num[0] = 3.016159211107498f,
    .num[1] = 6.032318422214996f,
    .num[2] = 3.016159211107498f,
    .den[0] = 1.0f,
    .den[1] = -1.486886029717710f,
    .den[2] = 0.487509692354036f,
};


float TF_2RD_step(float dt, float r)
{
    TF_2RD_h h = &g_tf_2rd;
    h->r = r;
    h->u = h->num[0] * h->r + h->num[1]*h->r_pre + h->num[2]*h->r_pre2
    - h->den[1] * h->u_pre - h->den[2] * h->u_pre2;
    h->r_pre2 = h->r_pre;
    h->r_pre = h->r;
    h->u_pre2 = h->u_pre;
    h->u_pre = h->u;
    return h->u;
}


