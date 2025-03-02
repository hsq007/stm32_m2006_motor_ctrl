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

void Friction_Identify_init(void);
void Friction_Identify_step(float dt);


int main(void)
{
    BSP_Iinitialization();
    EROB_DRV_init();
    MAIN_LOG("bsp init ok!");
    MAIN_LOG("bsp init ok!");
    MAIN_LOG("bsp init ok!");
    MAIN_LOG("start morot!");
    //  开机后进调试模式手动设置
    // Friction_Identify_init();
    
    while(1)
    {
        ;
    }
}

// 重力补偿
float k_gravity = 1670.0f;
float i_gravity;
// 摩擦力补偿
float spd_x[33] = {1.45f,2.18f,2.90f,3.63f,4.35f,5.08f,5.81f,6.53f,7.25f,7.98f,
                    8.70f,9.43f,10.15f,10.88f,11.63f,12.32f,13.05f,13.78f,14.50f,15.23f,
                    15.95f,16.68f,17.40f,18.13f,18.85f,19.58f,20.30f,21.03f,21.76f,22.48f,
                    23.25f,23.93f,24.65f};
float current_y[33] = {181.84f,203.89f,217.24f,229.78f,237.52f,246.98f,255.18f,264.07f,270.00f,277.13f,
                        280.96f,283.82f,288.64f,291.37f,297.72f,300.06f,304.34f,307.98f,313.61f,316.25f,
                        320.88f,322.75f,326.06f,330.71f,334.74f,338.79f,342.68f,348.19f,352.63f,357.74f,
                        356.20f,364.31f,368.71f};
float current_fric;
float fric_gain = 0.0f;
float gravity_gain = 0.0f;
// 这个线程不能被阻塞 1ms 执行一次
void app_1ms_task(void)
{
    float dt = 0.001f;
    // 重力补偿计算
    float angle = EROB_DRV_get_angle();
    i_gravity = k_gravity * sin(angle);
    // 摩擦力补偿计算
    float spd_fbk = EROB_DRV_get_speed_rpm();
    float dir = 0.0f;
    if(spd_fbk > 0.01f)
    {
        dir = 1.0f;
    }
    else if(spd_fbk < -0.01f)
    {
        dir = -1.0f;
    }
    else
    {
        dir = 0.0f;
    }
    current_fric = dir * HSQ_MATH_linear_table_interpolation(spd_x, current_y, 33u, HSQ_MATH_absf(spd_fbk));
    // Friction_Identify_step(dt);
    float current = fric_gain * current_fric + gravity_gain * i_gravity;
    EROB_DRV_set_ref(current);
    EROB_DRV_step(dt);
}






typedef struct{
    float dt; // 运行周期
    ERBO_DRV_mode_e mode; // 辨识模式
    uint8_t state; // 状态
    uint8_t start; // 开始测量 0x01
    float ref; // 目标值
    float ref_dir; // 方向
    uint8_t ref_idx; // 当前采集第几个点的数据
    float ref_min; // 最小目标值
    float ref_max; // 最大目标值
    uint8_t data_num; // 采集的数据点个数
    float spd; // 模组转速 rpm
    float current; // 电流 mA
    float angle_det; // 双编码器插值 deg
    // 采集平均值
    uint16_t cnt;
    float spd_avr; // 模组转速 rpm
    float current_avr; // 电流 mA
    float angle_det_avr; // 双编码器插值 deg
    float delay; // 延时 s
    float stable_time; // 稳定时间 s
    float stop_time; // 从最大转速停止的时间 s
}ST_FRIC_IDEN_t, *ST_FRIC_IDEN_h;

ST_FRIC_IDEN_t g_fric_ident;

void Friction_Identify_init(void)
{
    ST_FRIC_IDEN_h h = &g_fric_ident;
    h->data_num = 40u;
    h->ref_min = 0.0f;
    h->ref_max = 29.0f*HSQ_MATH_K_RPM2RADPS;
    h->stop_time = 20.0f;
    h->start = 0x00;
    h->stable_time = 5.0f;
    h->ref_dir = 1.0f;
    h->mode = EROB_DRV_MODE_SPD;
    EROB_DRV_set_mode(h->mode);
    EROB_DRV_set_ref(0.0f);
    EROB_DRV_set_motor_en(0x01);
}

// 辨识摩擦力模型-采集数据
void Friction_Identify_step(float dt)
{
    ST_FRIC_IDEN_h h = &g_fric_ident;
    h->dt = dt;
    h->spd = EROB_DRV_get_speed_rpm();
    h->current = EROB_DRV_get_current();
    h->angle_det = EROB_DRV_get_angle_det_deg();
    switch(h->state)
    {
        case 0x00: // 空闲
        {
            if(h->start)
            {
                h->state = 10u;
                h->start = 0x00;
                MAIN_LOG("%d, spd(rpm):%.3f, current(mA):%.3f, angle_det(deg):%.3f, current_fric(mA)", h->ref_idx, h->spd_avr, h->current_avr, h->angle_det_avr);
            }
            break;
        }

        case 10: // 设置目标转速 等待转速稳定
        {
            h->ref = h->ref_dir * (h->ref_min + h->ref_idx * (h->ref_max - h->ref_min) / (float)h->data_num);
            
            h->delay += h->dt;
            if(h->delay > h->stable_time)
            {
                h->delay = 0.0f;
                if(h->ref_idx < h->data_num)
                {
                    h->ref_idx ++;
                    h->state = 20u;
                }
                else
                {
                    h->ref_idx = 0x00;
                    h->ref = 0.0f;
                    if(h->ref_dir > 0.0f)
                    {
                        h->state = 30u;
                        h->ref_dir = -1.0f;
                    }
                    else
                    {
                        h->state = 0x00;
                    }
                }
            }
            break;
        }

        case 20: // 采集数据
        {
            
            h->spd_avr += h->spd;
            h->angle_det_avr += h->angle_det;
            h->current_avr +=h->current;
            h->cnt ++;
            uint16_t data_num = 5000u;
            if(h->cnt > data_num)
            {
                h->spd_avr /= data_num;
                h->angle_det_avr /= data_num;
                h->current_avr /= data_num;
                h->cnt = 0x00;
                h->state = 10u;
                MAIN_LOG("%d, %.3f, %.3f, %.3f,%.3f", h->ref_idx, h->spd_avr, h->current_avr, h->angle_det_avr, current_fric);
                h->spd_avr = 0.0f;
                h->angle_det_avr  = 0.0f;
                h->current_avr = 0.0f;
            }
            break;
        }

        case 30:// 减速到0
        {
            h->delay += h->dt;
            if(h->delay > h->stop_time)
            {
                h->delay = 0.0f;
                h->state = 10u;
            }
        }

        default:
            break;
    }
     EROB_DRV_set_ref(h->ref);
}

