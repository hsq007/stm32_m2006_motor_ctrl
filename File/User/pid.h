/*
 * @Author: hsq007 2267222816@qq.com
 * @Date: 2024-12-15 21:22:02
 * @LastEditors: hsq007 2267222816@qq.com
 * @LastEditTime: 2024-12-15 22:22:57
 * @FilePath: \20241214-M2006电机实验\File\User\pid.h
 * @Description: 并联型PID，和simulink中模型一致
 */
#ifndef __PID_H__
#define __PID_H__


#include "hsq_math.h"

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
    float err_pre;
    float u_kd_pre;
}PID_t, *PID_h;

void PID_init(PID_h h, float kp, float ki, float kd, float filter_n, float u_max);
float PID_step(PID_h h, float dt, float ref, float fbk);

#endif



