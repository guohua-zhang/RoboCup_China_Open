#include "bsp_motor.h"
#include "main.h"
#include "tim.h"

//使能定时器PWM通道
void motor_pwm_enable(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}


void motor_set_pwm(uint8_t motor, int32_t pid_out)
{
	int32_t ccr = 0;
	
	if (pid_out >= 0)
	{
		if (pid_out > MOTOR_PWM_MAX)
			ccr = MOTOR_PWM_MAX;
		else
			ccr = pid_out;
		
		switch (motor)
		{
			case 1: TIM1->CCR2 = 0; TIM1->CCR1 = ccr;	break;
			case 2: TIM1->CCR4 = 0; TIM1->CCR3 = ccr; break;
			case 3: TIM8->CCR2 = 0; TIM8->CCR1 = ccr;	break;
			case 4: TIM8->CCR4 = 0; TIM8->CCR3 = ccr;	break;
			default: ; //TODO
		}
	}
	
	else if (pid_out < 0)
	{
		if (pid_out < -MOTOR_PWM_MAX)
			ccr = MOTOR_PWM_MAX;
		else
			ccr = -pid_out;
		
		switch (motor)
		{
			case 1: TIM1->CCR1 = 0; TIM1->CCR2 = ccr;	break;
			case 2: TIM1->CCR3 = 0; TIM1->CCR4 = ccr;	break;
			case 3: TIM8->CCR1 = 0; TIM8->CCR2 = ccr;	break;
			case 4: TIM8->CCR3 = 0; TIM8->CCR4 = ccr;	break;
			default: ; //TODO
		}
	}
}


/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
**************************************************************************/
int read_encoder(uint8_t TIMX)
{
	int Encoder_TIM;    
	switch(TIMX)
	{
		case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0; /*L0step-=Encoder_TIM;*/
			break;
		case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0; /*L1step-=Encoder_TIM;*/
			break;	
		case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0; /*R0step+=Encoder_TIM;*/
			break;
		case 5:	 Encoder_TIM= (short)TIM5 -> CNT;  TIM5 -> CNT=0; /*R1step+=Encoder_TIM;*/
			break;
		default:  Encoder_TIM=0;
	}
	return Encoder_TIM;
}



