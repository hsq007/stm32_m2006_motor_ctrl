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
