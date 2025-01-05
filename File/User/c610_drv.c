#include "c610_drv.h"
#include "rtt.h"
#include "stm32f10x.h"
#include "hsq_math.h"

//#define RTT_SCOPR_USE

typedef struct
{
    uint8_t en_input;       // 使能输入 0x01=使能输入
    uint8_t update_zero;    // 更新零位
    float dt;               // 采样周期 s
    float current_ref;      // 电流目标值 A

    float current_fbk;      // 电机电流反馈 A

    // 减速箱前的参数
    float speed_fbk;        // 减速箱前的转速 rad/s
    float speed_fbk_rpm;    // 减速箱前的转速 rpm
    float pos_fbk;          // 减速箱前的机械位置 deg

    // 减速箱后的参数
    float speed_fbk_2;        // 减速箱后的转速 rad/s
    float speed_fbk_rpm_2;    // 减速箱后的转速 rpm
    float pos_fbk_2;          // 减速箱后的机械位置 deg

    // 连续的位置反馈
    float pos_fbk_sum;
    float speed_fbk_sum;

    // 配置参数
    uint8_t idx;            // 电调的ID，1/2/3/4
    float siGearRatio;      // 电机减速器减速比
	int16_t siNumber;       // 编码器线数
    
    // 协议解析用
    uint32_t can_rx_id;     // CAN接收报文的ID
    int16_t siSpeed;        // 电调反馈的转速 rpm
    int16_t siRawValue;     // 本次编码器的原始值
    int16_t siValue_pre;
    int16_t siValue_det;
    float   siNumber_div;   // 编码器线数的倒数
    int16_t siCurrent;      // 电调反馈的电流原始值
    float   siGearRatio_div; // 减速比的倒数

    // 发送控制指令用
    int16_t current_cmd;    // 电流指令值 -10000~10000
    uint8_t current_cmd_high;  // 电流指令高八位
    uint8_t current_cmd_low;   // 电流指令低八位
}C610_DRV_t, *C610_DRV_h;

C610_DRV_t g_c610_drv = {
    .idx = 0x01,
    .siGearRatio = 36.0f,
    .siNumber = 8192,
    .en_input = 0x01,
};

// 初始化电机参数
void C610_DRV_init(void)
{
    C610_DRV_h h = &g_c610_drv;
    h->siNumber_div = 1.0f / (float)h->siNumber;
    h->siGearRatio_div = 1.0f / (float)h->siGearRatio;
}

/**
 * 放到CNA的接收函数中解析数据
 * dt 运行周期 s
 * *data CAN报文中的数据
 * **/
void C610_DRV_rx_step(float dt, uint32_t message_id,  uint8_t data[])
{
    // 解析CAN报文中的位置速度电流反馈信号
    C610_DRV_h h = &g_c610_drv;
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
    h->pos_fbk = 360.0f * (float)h->siRawValue * h->siNumber_div;
    h->speed_fbk_rpm = h->siSpeed;
    h->speed_fbk = h->speed_fbk_rpm * HSQ_MATH_K_RPM2RADPS;

    h->pos_fbk_2 = 360.0f * (float)h->siRawValue * h->siNumber_div * h->siGearRatio_div;
    h->speed_fbk_rpm_2 = h->siSpeed * h->siGearRatio_div;
    h->speed_fbk_2 = h->speed_fbk_rpm * HSQ_MATH_K_RPM2RADPS * h->siGearRatio_div;

    int16_t si_det = (int16_t)h->siRawValue - (int16_t)h->siValue_pre;
    int16_t siNumber_div_2 = h->siNumber/2u;
    if(si_det > siNumber_div_2)
    {
        h->siValue_det = si_det - h->siNumber;
    }
    else if(si_det < -siNumber_div_2)
    {
        h->siValue_det = si_det + h->siNumber;
    }
    else
    {
        h->siValue_det = si_det;
    }
    if(0x00 == h->update_zero)
    {
        h->pos_fbk_sum = h->pos_fbk;
        h->update_zero = 0x01;
        h->siValue_det = 0x000; 
    }
    else
    {
        h->pos_fbk_sum += 360.0f * (float)h->siValue_det * h->siNumber_div;
        h->speed_fbk_sum =  60.0F * (float)h->siValue_det * h->siNumber_div / h->dt;
    }
    h->siValue_pre = h->siRawValue;
    

    // 打印波形
    #ifdef RTT_SCOPR_EN
    rtt_scope_write(h->siCurrent, h->siSpeed, h->siRawValue, 0x00, 0x00, 0x00);
    #endif
}

/**
 * std_id 发送的CAN_ID，对于1/2/3/4电机为0x200
 * cmd_mx 第1/2/3/4电机的电流指令值 -10000~10000对于-10~+10A
 * **/
CanTxMsg g_tx_message;
void CAN_send_current_cmd(uint32_t std_id, int16_t cmd_m1, int16_t cmd_m2, int16_t cmd_m3, int16_t cmd_m4)
{
    g_tx_message.StdId = std_id;
    g_tx_message.IDE = CAN_Id_Standard;
    g_tx_message.RTR = CAN_RTR_Data;
    g_tx_message.DLC = 0x08;
    g_tx_message.Data[0] = (uint8_t)(cmd_m1 >> 0x08);
    g_tx_message.Data[1] = (uint8_t) cmd_m1;
    g_tx_message.Data[2] = (uint8_t)(cmd_m2 >> 0x08);
    g_tx_message.Data[3] = (uint8_t) cmd_m2;
    g_tx_message.Data[4] = (uint8_t)(cmd_m3 >> 0x08);
    g_tx_message.Data[5] = (uint8_t) cmd_m3;
    g_tx_message.Data[6] = (uint8_t)(cmd_m4 >> 0x08);
    g_tx_message.Data[7] = (uint8_t) cmd_m4;
    CAN_Transmit(CAN1, &g_tx_message);
}


/**
 * 把电流指令解析为CAN报文格式
 * **/
void C610_DRV_tx_step(void)
{
    C610_DRV_h h = &g_c610_drv;
    h->current_cmd = h->current_ref * 0.1f * 10000.0f;
}


/**
 * 向C610电调发送电流控制指令
 * current_ref 电流目标值
 * 
 * **/
void C610_DRV_set_current_ref(float current_ref)
{
    C610_DRV_h h = &g_c610_drv;
    if(h->en_input)
    {
        h->current_ref = current_ref;
    }
}

float C610_DRV_get_current_ref(void)
{
    C610_DRV_h h = &g_c610_drv;
    return h->current_ref;
}

int16_t C610_DRV_get_current_cmd(void)
{
    C610_DRV_h h = &g_c610_drv;
    return h->current_cmd;
}

// 反馈速度
float C610_DRV_get_speed(void)
{
    C610_DRV_h h = &g_c610_drv;
    return h->speed_fbk_rpm;
}

// 反馈位置
float C610_DRV_get_pos(void)
{
    C610_DRV_h h = &g_c610_drv;
    return h->pos_fbk;
}


// 获取连续的位置反馈
float C610_DRV_get_pos_sum(void)
{
    C610_DRV_h h = &g_c610_drv;
    return h->pos_fbk_sum;
}
