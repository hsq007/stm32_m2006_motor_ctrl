#include "motor_ctrl.h"
#include "pid.h"
#include "c610_drv.h"
#include "freq_scan.h"
#include "tf_2rd.h"

#define SPEED_PID_KP (0.00509732584143159F)
#define SPEED_PID_KI (0.108604567917919F)
#define SPEED_PID_KD (1.90224160971236e-05)
#define SPEED_PID_N  (321.919119694805F)
#define SPEED_PID_UM (2.0F)

PID_t g_pid;
PID_t g_pid_2;
TF_2RD_t g_tf_2rd;

void MOTOR_CTRL_init(void)
{
    C610_DRV_init();
    FREQ_SCAN_init();
    TF_2RD_init(&g_tf_2rd, 3.016159211107498f, 6.032318422214996f,
                3.016159211107498f, 1.0f, -1.486886029717710f, 0.487509692354036f);
    PID_init(&g_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_N, SPEED_PID_UM);
    PID_init(&g_pid_2, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_N, SPEED_PID_UM);
}

// 对实际的电机进行PID闭环控制
float speed_ref = 0.0f;
void MOTRO_CTRL_step(float dt)
{
    float speed_fbk = C610_DRV_get_speed();
    float pid_u = PID_step(&g_pid, dt, speed_ref, speed_fbk);
    C610_DRV_set_current_ref(pid_u);
}

// 对电机的传递函数进行PID控制
void MOTOR_SIMULATE_step(float dt)
{
    // PID 控制辨识的速度环模型
    float speed_est = TF_2RD_get_out(&g_tf_2rd);
    float pid_u2 = PID_step(&g_pid_2, 0.001f, speed_ref, speed_est);
    TF_2RD_step(&g_tf_2rd, dt, pid_u2);
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


