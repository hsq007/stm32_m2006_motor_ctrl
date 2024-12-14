#include "stm32f10x.h"


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

