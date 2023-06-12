#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "sys.h"

#define MOTOR_PWM_MAX 6000

//使能定时器PWM通道
void motor_pwm_enable(void);

//电机PWM设置函数
//motor: 电机编号
//pid_out: 定时器CCR值，最大为7200，函数内限幅MOTOR_PWM_MAX，正值为正转，负值为反转
void motor_set_pwm(uint8_t motor, int32_t pid_out);

int read_encoder(uint8_t TIMX);

#endif

