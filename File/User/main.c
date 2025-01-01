/*
 * @Author: hsq007 2267222816@qq.com
 * @Date: 2024-12-09 22:36:44
 * @LastEditors: hsq007 2267222816@qq.com
 * @LastEditTime: 2024-12-16 02:29:02
 * @FilePath: \20241214-M2006电机实验\File\User\main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "stm32f10x.h"
#include "c610_drv.h"
#include "bsp_init.h"
#include "hsq_math.h"
#include "stdio.h"
#include "freq_scan.h"
#include "tf_2rd.h"
#include "pid.h"


PID_t g_pid_p = {.kp = 1.0f,.ki = 0.0f,.kd = 0.0f,.filter_n = 0.0f};
PID_t g_pid_i = {.kp = 0.0f,.ki = 1.0f,.kd = 0.0f,.filter_n = 0.0f};
PID_t g_pid_d = {.kp = 0.0f,.ki = 0.0f,.kd = 1.0f,.filter_n = 10.0f};
float ref = 1.0f;
int main_2(void)
{
    while(1)
    {
        PID_step(&g_pid_p, 0.001f, ref, 0.0f);
        PID_step(&g_pid_i, 0.001f, ref, 0.0f);
        PID_step(&g_pid_d, 0.001f, ref, 0.0f);
//        delay_ms(1);
    }
}

#define MAIN_LOG(format, ...) printf("[MAIN]" format"\r\n", ##__VA_ARGS__)

#define SPEED_PID_KP (0.00509732584143159F)
#define SPEED_PID_KI (0.108604567917919F)
#define SPEED_PID_KD (1.90224160971236e-05)
#define SPEED_PID_N  (321.919119694805F)

PID_t g_pid = {
    .kp = SPEED_PID_KP,
    .ki = SPEED_PID_KI,
    .kd = SPEED_PID_KD,
    .filter_n = SPEED_PID_N,
    .en_fbk = 0x01,
    .en_input = 0x01,
    .u_max = 2.0f,
};

PID_t g_pid_2 = {
    .kp = SPEED_PID_KP,
    .ki = SPEED_PID_KI,
    .kd = SPEED_PID_KD,
    .filter_n = SPEED_PID_N,
    .en_fbk = 0x01,
    .en_input = 0x01,
    .u_max = 2.0f,
};



int main(void)
{
    BSP_Iinitialization();
    C610_DRV_init();
    FREQ_SCAN_init();
    MAIN_LOG("bsp init ok!");
    MAIN_LOG("bsp init ok!");
    MAIN_LOG("bsp init ok!");
    MAIN_LOG("bsp init ok!");
    
    while(1)
    {
        ;
    }
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


/*
函 数 名：USB_LP_CAN1_RX0_IRQHandler
函数功能：CAN1接收中断
备    注：处理电调反馈数据
*/
CanRxMsg g_rx_message;
float speed_ref = 0.0f;
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &g_rx_message);
        // 解析电调反馈数据
        C610_DRV_rx_step(0.001f, g_rx_message.StdId, g_rx_message.Data);
        // 发送电调电流指令
        C610_DRV_tx_step();
        int16_t cmd_m1 = C610_DRV_get_current_cmd();
        CAN_send_current_cmd(0x200, cmd_m1, 0x00, 0x00, 0x00);
        
        // 扫频
        float speed_fbk = C610_DRV_get_speed();
        FREQ_SCAN_set_output(speed_fbk);
        FREQ_SCAN_step(0.001f);
        
        // 系统辨识结果
        TF_2RD_step(0.001f);
        
        // PID 控制实际电机的速度环
        // float scan_u = FREQ_SCAN_get_input();
        float pid_u = PID_step(&g_pid, 0.001f, speed_ref, speed_fbk);
        C610_DRV_set_current_ref(pid_u);

        // PID 控制辨识的速度环模型
        float speed_est = TF_2RD_get_out();
        float pid_u2 = PID_step(&g_pid_2, 0.001f, speed_ref, speed_est);

        float c610_ref = C610_DRV_get_current_ref();
        TF_2RD_set_input(pid_u2);
        
    }
}


/*
函 数 名：TIM2_IRQHandler
函数功能：TIM2定时器中断服务函数
备    注：工作频率200Hz  PID控制频率200Hz
*/
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
