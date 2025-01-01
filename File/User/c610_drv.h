/*
 * @Author: hsq007 2267222816@qq.com
 * @Date: 2024-12-14 22:21:31
 * @LastEditors: hsq007 2267222816@qq.com
 * @LastEditTime: 2024-12-15 21:10:49
 * @FilePath: \20241214-M2006电机实验\File\User\c610.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef M610_DRV_H__
#define M610_DRV_H__

#include "hsq_math.h"

void C610_DRV_init(void);
void C610_DRV_rx_step(float dt, uint32_t message_id,  uint8_t data[]);
int16_t C610_DRV_get_current_cmd(void);
void C610_DRV_tx_step(void);
float C610_DRV_get_speed(void);
float C610_DRV_get_pos(void);
void C610_DRV_set_current_ref(float current_ref);
float C610_DRV_get_current_ref(void);
void CAN_send_current_cmd(uint32_t std_id, int16_t cmd_m1, int16_t cmd_m2, int16_t cmd_m3, int16_t cmd_m4);

#endif

