/*
 * @Author: hsq007 2267222816@qq.com
 * @Date: 2024-12-15 21:22:02
 * @LastEditors: hsq007 2267222816@qq.com
 * @LastEditTime: 2024-12-15 21:41:42
 * @FilePath: \20241214-M2006电机实验\File\User\pid.h
 * @Description: 并联型PID，和simulink中模型一致
 */
#ifndef __PID_H__
#define __PID_H__


#include "hsq_math.h"
float PID_step(float dt, float ref, float fbk);

#endif



