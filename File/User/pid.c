/*
 * @Author: hsq007 2267222816@qq.com
 * @Date: 2024-12-15 21:21:58
 * @LastEditors: hsq007 2267222816@qq.com
 * @LastEditTime: 2024-12-15 21:50:50
 * @FilePath: \20241214-M2006电机实验\File\User\pid.c
 * @Description: 并联型PID，和simulink中模型一致
 */


#include "pid.h"

typedef struct{
    // 控制参数
    uint8_t en_input; // 0x01=使能输入
    uint8_t en_fbk; // 0x01=使能反馈
    float dt; // 控制周期 s
    float kp; // 比例
    float ki; // 积分
    float kd; // 微分
    float filter_n; // 微分项滤波系数 微分项最大输出值为 N*kd
    float u_max; // 输出限幅

    // 输入输出
    float ref;
    float fbk;
    float err;
    float u;
    
    // 中间变量
    float u_kp; 
    float u_ki;
    float u_kd;
    float ref_pre;
    float ref_pre2;
    float u_pre;
    float u_pre2;
}PID_t, *PID_h;


PID_t g_pid = {
    .kp = 1.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .en_fbk = 0x01,
    .en_input = 0x01,
    .filter_n = 100.0f,
    .u_max = 2.0f,
};

float PID_step(float dt, float ref, float fbk)
{
    PID_h h = &g_pid;
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
