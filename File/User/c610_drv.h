/*
 * @Author: hsq007 2267222816@qq.com
 * @Date: 2024-12-14 22:21:31
 * @LastEditors: hsq007 2267222816@qq.com
 * @LastEditTime: 2024-12-14 23:55:14
 * @FilePath: \20241214-M2006电机实验\File\User\c610.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef M610_DRV_H__
#define M610_DRV_H__

#include "hsq_math.h"

typedef struct
{
    uint8_t en_input;       // 使能输入 0x01=使能输入
    float dt;               // 采样周期 s
    float current_ref;      // 电流目标值 A
    float speed_fbk;        // 电机减速器输出轴转速 rad/s
    float speed_fbk_rpm;    // 电机减速器输出轴转速 rpm
	float current_fbk;      // 电机电流反馈 A
    float pos_fbk;          // 电机当前机械位置 deg

    // 配置参数
    uint8_t idx;            // 电调的ID，1/2/3/4
    float siGearRatio;      // 电机减速器减速比
	int16_t siNumber;       // 编码器线数
    
    // 协议解析用
    uint32_t can_rx_id;     // CAN接收报文的ID
    int16_t siSpeed;        // 电调反馈的转速 rpm
    int16_t siRawValue;     // 本次编码器的原始值
    float   siNumber_div;   // 编码器线数的倒数
    int16_t siCurrent;      // 电调反馈的电流原始值
    float   siGearRatio_div; // 减速比的倒数

    // 发送控制指令用
    int16_t current_cmd;    // 电流指令值 -10000~10000
    uint8_t current_cmd_high;  // 电流指令高八位
    uint8_t current_cmd_low;   // 电流指令低八位
}C610_DRV_DRV_t, *C610_DRV_DRV_h;

void C610_DRV_init(C610_DRV_DRV_h h, uint8_t idx, float gear_ratio, int16_t si_num);
void C610_DRV_rx_step(C610_DRV_DRV_h h, float dt, uint32_t message_id,  uint8_t data[]);
int16_t C610_DRV_get_current_cmd(C610_DRV_DRV_h h);
void C610_DRV_DRV_tx_step(C610_DRV_DRV_h h);
float C610_DRV_get_speed(C610_DRV_DRV_h h);
float C610_DRV_get_pos(C610_DRV_DRV_h h);

#endif

