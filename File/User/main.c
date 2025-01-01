/*
 * @Author: hsq007 2267222816@qq.com
 * @Date: 2024-12-09 22:36:44
 * @LastEditors: hsq007 2267222816@qq.com
 * @LastEditTime: 2024-12-16 02:29:02
 * @FilePath: \20241214-M2006电机实验\File\User\main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "stm32f10x.h"
#include "bsp_init.h"
#include "stdio.h"
#include "motor_ctrl.h"

#define MAIN_LOG(format, ...) printf("[MAIN]" format"\r\n", ##__VA_ARGS__)


int main(void)
{
    BSP_Iinitialization();
    MOTOR_CTRL_init();
    MAIN_LOG("bsp init ok!");
    MAIN_LOG("bsp init ok!");
    MAIN_LOG("bsp init ok!");
    MAIN_LOG("bsp init ok!");
    
    while(1)
    {
        ;
    }
}

// 这个线程不能被阻塞 1ms 执行一次
void app_1ms_task(void)
{
    float dt = 0.001f;
    MOTRO_CTRL_step(dt);
    MOTOR_SIMULATE_step(dt);
    // MOTOR_SCAN_step(dt);
}
