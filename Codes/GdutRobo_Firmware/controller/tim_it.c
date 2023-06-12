#include "tim_it.h"
#include "main.h"
#include "stdio.h"
#include "tim.h"
#include "pid.h"
#include "bsp_motor.h"
#include "bsp_led.h"
#include "speed_ctrl.h"
#include "scaner.h"
#include "sin_generate.h"
#include "bsp_linefollower.h"
#include "turn.h"
#include "my_print.h"

#define PI  3.1415926535

#define Speed_Bias_Up 10
#define Speed_Bias_Down 10

static uint32_t tim6_tick = 0;
uint32_t const *TIM6_tick = &tim6_tick;
volatile uint8_t PIDMode;
uint8_t line_gyro_switch = 0;

struct sin_param motor_sin_param = {.time = 0, .gain = 80, .angular_velocity = 0.3};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim6))
    {
			++tim6_tick;
		
		if (*TIM6_tick % 100 == 0)
			LED_twinkle();
		
		//周期10ms
		if (*TIM6_tick % 2 == 0)
		{
			motor_L0.measure = read_encoder(2);
			motor_L1.measure = read_encoder(3);
			motor_R0.measure = read_encoder(4);
			motor_R1.measure = read_encoder(5);
			
			//四个轮子编码器读取的脉冲数量的平均值
			motor_all.encoder_avg += (motor_L0.measure + motor_L1.measure + motor_R0.measure + motor_R1.measure) / 4;
			
			//轮径7.2cm
			//总路程 = （平均脉冲数 * πd） / 每个轮子转一圈返回的脉冲数
			motor_all.Distance = (motor_all.encoder_avg * 7.2 * PI) / 2245.0f;
			
			//陀螺仪自平衡->循迹
			if (line_gyro_switch == 1)    //这里的line_gyro_switch是在PIDMODE切换情况下所产生的标志位
			{
				line_pid_obj = gyroG_pid;
				TC_speed = TG_speed;
				gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
				TG_speed = (struct Gradual){0,0,0};
				line_gyro_switch = 0;
			}
			//循迹->陀螺仪自平衡
			else if (line_gyro_switch == 2)
			{
				gyroG_pid = line_pid_obj;
				TG_speed = TC_speed;
				line_pid_obj = (struct P_pid_obj){0,0,0,0,0,0,0};
				TC_speed = (struct Gradual){0,0,0};
				line_gyro_switch = 0;
			}
			else
			{
				if (PIDMode == is_Line)
				{
					gradual_cal(&TC_speed, motor_all.Cspeed, motor_all.Cincrement);
					Go_Line(TC_speed.Now);
				}
				else
					motor_all.Cspeed = 0;
				
				//转弯PID控制
				if (PIDMode == is_Turn)	
				{
					if (Turn_Angle(angle.AngleT))
					{
						gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0};
					}
				}

				//自平衡PID控制
				if (PIDMode == is_Gyro)
				{ 
					gradual_cal(&TG_speed, motor_all.Gspeed, motor_all.Gincrement);	
					runWithAngle(angle.AngleG, TG_speed.Now);
				}
				else
					motor_all.Gspeed = 0;
			}
			
			motor_L0.target = motor_L1.target = motor_all.Lspeed;
			motor_R0.target = motor_R1.target = motor_all.Rspeed;
			
			incremental_PID(&motor_L0, &motor_pid_param);
			incremental_PID(&motor_L1, &motor_pid_param);
			incremental_PID(&motor_R0, &motor_pid_param);
			incremental_PID(&motor_R1, &motor_pid_param);
			
			motor_set_pwm(1, (int32_t)motor_L0.output);
			motor_set_pwm(2, (int32_t)motor_L1.output);
			motor_set_pwm(3, (int32_t)motor_R0.output);
	  	motor_set_pwm(4, (int32_t)motor_R1.output);
			
	//		if (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)))
	//		{
	//			//printf("%d, %d, %d, %d\r\n", motor_L0.target, motor_L1.target, motor_R0.target, motor_R1.target);
				//
//	printf("%d,%d,%d,%d,%d\r\n", motor_L0.target, motor_L0.measure, motor_L1.measure, motor_R0.measure, motor_R1.measure);
				//printf("%f\r\n",motor_all.Distance);
			//printf("%d",motor_R0.measure);
	//			//printf("%f,%f,%f,%f\r\n", motor_L0.output, motor_L1.output, motor_R0.output, motor_R1.output);
	//		}
				//printf("%d\n",Infrared_FrontLeft);
		}
		
//		if (*TIM6_tick % 4 == 0)
//			printf("%d\r\n", (int)get_pitch());
		
		//周期5ms
		LFB_send();  //查询循迹板
    }
}


void pid_mode_switch(uint8_t target_mode)
{
	switch (target_mode)
	{
		case is_Turn: {
			gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			TG_speed = (struct Gradual){0,0,0};
			line_pid_obj = (struct P_pid_obj){0,0,0,0,0,0,0};
			TC_speed = (struct Gradual){0,0,0};
			gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			break;
		}
		
		case is_Line: {
			if (PIDMode == is_Gyro)  //从自平衡切换到循线
			{
				line_gyro_switch = 1;
			}
			else
			{
				gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			}
			break;
		}
		
		case is_Gyro: {
			if (PIDMode == is_Line)  //从循线切换到自平衡
			{
				line_gyro_switch = 2;
			}
			else
			{
				gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			}
			break;
		}
		
		case is_Free: {
			break;
		}
		
		case is_No: {
			line_pid_obj = (struct P_pid_obj){0,0,0,0,0,0,0};
			TC_speed = (struct Gradual){0,0,0};
			gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			TG_speed = (struct Gradual){0,0,0};
			gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			break;
		}
	}
	PIDMode = target_mode;
}





