#include "motor_ctrl.h"
#include "pid.h"
#include "c610_drv.h"
#include "freq_scan.h"
#include "tf_2rd.h"

#define SPEED_PID_KP (0.00509732584143159F)
#define SPEED_PID_KI (0.108604567917919F)
#define SPEED_PID_KD (1.90224160971236e-05)
#define SPEED_PID_N  (321.919119694805F)

PID_t g_pid = {
    .kp = SPEED_PID_KP,
    .ki = SPEED_PID_KI,
    .kd = SPEED_PID_KD,
    .filter_n = SPEED_PID_N,
    .en_fbk = 0x01,
    .en_input = 0x01,
    .u_max = 2.0f,
};

PID_t g_pid_2 = {
    .kp = SPEED_PID_KP,
    .ki = SPEED_PID_KI,
    .kd = SPEED_PID_KD,
    .filter_n = SPEED_PID_N,
    .en_fbk = 0x01,
    .en_input = 0x01,
    .u_max = 2.0f,
};

void MOTOR_CTRL_init(void)
{
    C610_DRV_init();
    FREQ_SCAN_init();
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
    // 系统辨识结果
    TF_2RD_step(dt);
    // PID 控制辨识的速度环模型
    float speed_est = TF_2RD_get_out();
    float pid_u2 = PID_step(&g_pid_2, 0.001f, speed_ref, speed_est);
    TF_2RD_set_input(pid_u2);
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


