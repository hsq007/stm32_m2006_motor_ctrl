/*
 * @Author: hsq007 2267222816@qq.com
 * @Date: 2024-12-09 22:36:44
 * @LastEditors: hsq007 2267222816@qq.com
 * @LastEditTime: 2024-12-14 23:59:19
 * @FilePath: \20241214-M2006电机实验\File\User\main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "stm32f10x.h"
#include "c610_drv.h"
#include "bsp_init.h"
#include "hsq_math.h"

C610_DRV_DRV_t g_C610_DRV_drv;

int main(void)
{
    BSP_Iinitialization();
    C610_DRV_init(&g_C610_DRV_drv, 0x01, 32.0f, 8192u);
    while(1)
    {
        ;
    }
}

/**
 * std_id 发送的CAN_ID，对于1/2/3/4电机为0x200
 * cmd_mx 第1/2/3/4电机的电流指令值 -10000~10000对于-10~+10A
 * **/
CanRxMsg g_rx_message;
void CAN_send_current_cmd(uint32_t std_id, int16_t cmd_m1, int16_t cmd_m2, int16_t cmd_m3, int16_t cmd_m4)
{
    g_rx_message.StdId = std_id;
    g_rx_message.IDE = CAN_Id_Standard;
    g_rx_message.RTR = CAN_RTR_Data;
    g_rx_message.DLC = 0x08;
    g_rx_message.Data[0] = (uint8_t)(cmd_m1 >> 0x08);
    g_rx_message.Data[1] = (uint8_t) cmd_m1;
    g_rx_message.Data[2] = (uint8_t)(cmd_m2 >> 0x08);
    g_rx_message.Data[3] = (uint8_t) cmd_m2;
    g_rx_message.Data[4] = (uint8_t)(cmd_m3 >> 0x08);
    g_rx_message.Data[5] = (uint8_t) cmd_m3;
    g_rx_message.Data[6] = (uint8_t)(cmd_m4 >> 0x08);
    g_rx_message.Data[7] = (uint8_t) cmd_m4;
}


/*
函 数 名：USB_LP_CAN1_RX0_IRQHandler
函数功能：CAN1接收中断
备    注：处理电调反馈数据
*/
CanTxMsg g_g_rx_message;
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &g_rx_message);
        // 解析电调反馈数据
        C610_DRV_rx_step(&g_C610_DRV_drv, 0.001f, g_rx_message.StdId, g_rx_message.Data);
        // 发送电调电流指令
        C610_DRV_DRV_tx_step(&g_C610_DRV_drv);
        int16_t cmd_m1 = C610_DRV_get_current_cmd(&g_C610_DRV_drv);
        CAN_send_current_cmd(0x200, cmd_m1, 0x00, 0x00, 0x00);
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
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );
    }
}

