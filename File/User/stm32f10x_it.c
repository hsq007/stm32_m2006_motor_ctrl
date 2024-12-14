#include "stm32f10x.h"


/*************************************************************************
�� �� ����USB_HP_CAN1_TX_IRQHandler
�������ܣ�CAN1�����ж�
��    ע��
*************************************************************************/
void USB_HP_CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }
}

