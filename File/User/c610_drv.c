#include "c610_drv.h"


// 初始化电机参数
void C610_DRV_init(C610_DRV_DRV_h h, uint8_t idx, float gear_ratio, int16_t si_num)
{
    h->idx = idx;
    h->siGearRatio = gear_ratio;
    h->siNumber = si_num;
    h->siNumber_div = 1.0f / (float)h->siNumber;
    h->siGearRatio_div = 1.0f / (float)h->siGearRatio;
}

/**
 * 放到CNA的接收函数中解析数据
 * dt 运行周期 s
 * *data CAN报文中的数据
 * **/
void C610_DRV_rx_step(C610_DRV_DRV_h h, float dt, uint32_t message_id,  uint8_t data[])
{
    // 解析CAN报文中的位置速度电流反馈信号
    h->dt = dt;
    h->can_rx_id = 0x200 + h->idx;
    
    // 解析报文
    if(h->can_rx_id == message_id)
    {
        h->siRawValue = (int16_t)(data[0] << 0x08 | data[1]);
        h->siSpeed = (int16_t)(data[2] << 0x08 | data[3]);
        h->siCurrent = (int16_t)(data[4] << 0x08 | data[5]);
    }
    // 换算为实际的单位
    h->current_fbk = 0.001f * h->siCurrent;
    h->pos_fbk = 360.0f * (float)h->siRawValue * h->siNumber_div * h->siGearRatio_div;
    h->speed_fbk_rpm = h->siSpeed;
    h->speed_fbk = h->speed_fbk_rpm * HSQ_MATH_K_RPM2RADPS;
}


/**
 * 把电流指令解析为CAN报文格式
 * data_high 高八位电流
 * data_low 低八位电流
 * **/
void C610_DRV_DRV_tx_step(C610_DRV_DRV_h h)
{
    h->current_cmd = h->current_ref * 0.1f * 10000.0f;
}


/**
 * 向C610电调发送电流控制指令
 * current_ref 电流目标值
 * 
 * **/
void C610_DRV_set_current_ref(C610_DRV_DRV_h h, float current_ref)
{
    h->current_ref = current_ref;
}

int16_t C610_DRV_get_current_cmd(C610_DRV_DRV_h h)
{
    return h->current_cmd;
}

// 反馈速度
float C610_DRV_get_speed(C610_DRV_DRV_h h)
{
    return h->speed_fbk_rpm;
}

// 反馈位置
float C610_DRV_get_pos(C610_DRV_DRV_h h)
{
    return h->pos_fbk;
}
