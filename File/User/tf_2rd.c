/*
 * @Author: hsq007 2267222816@qq.com
 * @Date: 2024-12-15 20:50:21
 * @LastEditors: hsq007 2267222816@qq.com
 * @LastEditTime: 2024-12-15 22:26:51
 * @FilePath: \20241214-M2006电机实验\File\User\tf.c
 * @Description: 有两个延迟的脉冲传递函数实现
 */
#include "tf_2rd.h"

void TF_2RD_init(TF_2RD_h h, float num_0, float num_1, float num_2, float den_0, float den_1, float den_2)
{
    h->num[0] = num_0;
    h->num[1] = num_1;
    h->num[2] = num_2;
    h->den[0] = den_0;
    h->den[1] = den_1;
    h->den[2] = den_2;
    h->en_input = 0x01;
}

float TF_2RD_step(TF_2RD_h h, float dt, float r)
{
    if(h->en_input)
    {
        h->r = r;
    }
    h->u = h->num[0] * h->r + h->num[1]*h->r_pre + h->num[2]*h->r_pre2
    - h->den[1] * h->u_pre - h->den[2] * h->u_pre2;
    h->r_pre2 = h->r_pre;
    h->r_pre = h->r;
    h->u_pre2 = h->u_pre;
    h->u_pre = h->u;
    return h->u;
}

float TF_2RD_get_out(TF_2RD_h h)
{
    return h->u;
}


