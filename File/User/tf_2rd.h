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

float TF_2RD_step(float dt);
float TF_2RD_get_out(void);
void TF_2RD_set_input(float input);

#endif



