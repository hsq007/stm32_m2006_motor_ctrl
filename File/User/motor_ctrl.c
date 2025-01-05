#include "motor_ctrl.h"
#include "pid.h"
#include "c610_drv.h"
#include "freq_scan.h"
#include "tf_2rd.h"
#include "integrator.h"

#define SPEED_PID_KP (0.00509732584143159F)
#define SPEED_PID_KI (0.108604567917919F)
#define SPEED_PID_KD (1.90224160971236e-05)
#define SPEED_PID_N  (321.919119694805F)
#define SPEED_PID_UM (10.0F)

#define POS_PID_KP (0.15024830058169F)
#define POS_PID_KI (0.264314415833988F)
#define POS_PID_KD (-0.00451345977176125F)
#define POS_PID_N  (32.7156063233189F)
#define POS_PID_UM (1000.0F)

PID_t g_pid_speed;
PID_t g_pid_speed_2;
PID_t g_pid_pos;
PID_t g_pid_pos_2;
TF_2RD_t g_tf_2rd;
INTEGRATOR_t g_integ;


void MOTOR_CTRL_init(void)
{
    C610_DRV_init();
    FREQ_SCAN_init();
    TF_2RD_init(&g_tf_2rd, 3.016159211107498f, 6.032318422214996f,
                3.016159211107498f, 1.0f, -1.486886029717710f, 0.487509692354036f);
    PID_init(&g_pid_speed, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_N, SPEED_PID_UM);
    PID_init(&g_pid_speed_2, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_N, SPEED_PID_UM);
    PID_init(&g_pid_pos, POS_PID_KP, POS_PID_KI, POS_PID_KD, POS_PID_N, POS_PID_UM);
    PID_init(&g_pid_pos_2, POS_PID_KP, POS_PID_KI, POS_PID_KD, POS_PID_N, POS_PID_UM);
    INTEGRATOR_init(&g_integ, HSQ_MATH_K_RAD2DEG,  0.0F);
    g_pid_pos.en_output = 0x00;
}

// 对实际的电机进行PID闭环控制
float pos_ref = 0.0f;
void MOTRO_CTRL_step(float dt)
{
    float speed_fbk = C610_DRV_get_speed();
    float pos_fbk = C610_DRV_get_pos_sum();
    float speed_ref = PID_step_2(&g_pid_pos, dt, pos_ref, pos_fbk);
    float current_ref = PID_step_2(&g_pid_speed, dt, speed_ref, speed_fbk);
    C610_DRV_set_current_ref(current_ref);
}

// 对电机的传递函数进行PID控制
void MOTOR_SIMULATE_step(float dt)
{
    static float current_ref = 0.0f;
    // 电机模型
    float speed_fbk = TF_2RD_step(&g_tf_2rd, dt, current_ref);
    float pos_fbk = INTEGRATOR_step(&g_integ, dt, speed_fbk);
    // PID 控制辨识的速度环模型
    float speed_ref = PID_step_2(&g_pid_pos_2, dt, pos_ref, pos_fbk);
    current_ref = PID_step_2(&g_pid_speed_2, dt, speed_ref, speed_fbk);
}

// 扫频控制
void MOTOR_SCAN_step(float dt)
{
    float speed_fbk = C610_DRV_get_speed();
    FREQ_SCAN_set_output(speed_fbk);
    FREQ_SCAN_step(dt);
    float input = FREQ_SCAN_get_input();
    C610_DRV_set_current_ref(input);
}


