// 零差云控合作项目 谐波模组建模
// 实现CAN自定义通信协议和重力补偿模型
// 摩擦力模型正在开发中
// 2025.02.23 by 黄胜全

#include "stm32f10x.h"
#include "bsp_init.h"
#include "stdio.h"
#include "erob_drv.h"
#include "math.h"

#define MAIN_LOG(format, ...) printf("[MAIN]" format"\r\n", ##__VA_ARGS__)


int main(void)
{
    BSP_Iinitialization();
    EROB_DRV_init();
    MAIN_LOG("bsp init ok!");
    MAIN_LOG("bsp init ok!");
    MAIN_LOG("bsp init ok!");
    MAIN_LOG("start morot!");
    //  开机后进调试模式手动设置
//    EROB_DRV_set_mode(EROB_DRV_MODE_SPD);
//    EROB_DRV_set_ref(1.0f);
//    EROB_DRV_set_motor_en(0x01);
    
    while(1)
    {
        ;
    }
}


float k_gravity = -1100.0f;
float i_gravity;
// 这个线程不能被阻塞 1ms 执行一次
void app_1ms_task(void)
{
    float dt = 0.001f;
    float angle = EROB_DRV_get_angle();
    i_gravity = k_gravity * sin(angle);
    EROB_DRV_set_ref(i_gravity);
    EROB_DRV_step(dt);
}
