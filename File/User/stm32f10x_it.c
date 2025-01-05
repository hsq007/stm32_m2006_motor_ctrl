#include "stm32f10x.h"
#include "c610_drv.h"
#include "motor_ctrl.h"

extern void app_1ms_task(void);

/*************************************************************************
函 数 名：USB_HP_CAN1_TX_IRQHandler
函数功能：CAN1发送中断
备    注：
*************************************************************************/
void USB_HP_CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
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
        // 低频APP执行
        app_1ms_task();
    }
}

/*
函 数 名：USB_LP_CAN1_RX0_IRQHandler
函数功能：CAN1接收中断
备    注：处理电调反馈数据
*/
CanRxMsg g_rx_message;
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
    }
}

