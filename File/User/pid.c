/*
 * @Author: hsq007 2267222816@qq.com
 * @Date: 2024-12-15 21:21:58
 * @LastEditors: hsq007 2267222816@qq.com
 * @LastEditTime: 2024-12-15 22:14:59
 * @FilePath: \20241214-M2006电机实验\File\User\pid.c
 * @Description: 并联型PID，和simulink中模型一致
 */

#include "pid.h"

float PID_step(PID_h h, float dt, float ref, float fbk)
{
    h->dt = dt;
    if(h->en_input)
    {
        h->ref = ref;
    }
    if(h->en_fbk)
    {
        h->fbk = fbk;
    }
    h->err = h->ref - h->fbk;
    h->u_kp = h->kp * h->err;

    // ki为0时清空积分值
    if(h->ki > 0.001f)
    {
        h->u_ki += h->ki * h->dt * h->err;
        h->u_ki = HSQ_MATH_limit(h->u_max, h->u_ki);
    }
    else
    {
        h->u_ki = 0.0f;
    }
    
    h->u_kd = h->kd * h->filter_n * (h->ref - h->ref_pre) - (h->filter_n * h->dt -1.0f) * h->u_pre;
    h->u = h->u_kp + h->u_ki + h->u_kd;

    // 历史值
    h->ref_pre2 = h->ref_pre;
    h->ref_pre = h->ref;
    h->u_pre2 = h->u_pre;
    h->u_pre = h->u;
    return h->u;
}