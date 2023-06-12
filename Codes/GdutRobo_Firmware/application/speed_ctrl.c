#include "speed_ctrl.h"
#include "tim_it.h"
#include "pid.h"

volatile struct Motors motor_all = {
	.Lspeed = 0,
	.Rspeed = 0,
	.encoder_avg = 0,
	.GyroG_speedMax = 50,
	.GyroT_speedMax = 80,
	.Cincrement = 1.5,	//循迹加速度
	.Gincrement = 1.5,		//非循迹加速度
	
	.is_UP = false,
	.is_DOWM = false,
};

struct Gradual TC_speed = {0,0,0}, TG_speed = {0,0,0};


void CarBrake(void)
{
	pid_mode_switch(is_No);
	motor_all.Lspeed = motor_all.Rspeed = 0;
}

//以一次函数缓慢加速或者缓慢停止
void gradual_cal(struct Gradual *gradual, float target, float increment)
{
	uint8_t direction = 0;
	
	if(target - gradual->Now < 0)
		direction = 0;
	else
		direction = 1;
	
	if(gradual->Now != target)
	{
		if (direction)
			gradual->Now += increment;
		else
			gradual->Now -= increment;
	}
	else 
	{	
		return;
	}
	
	if(direction == 1)
	{
		if(gradual->Now > target)
		{
			gradual->Now = target;
		}
	}
	else if(direction == 0)
	{
		if(gradual->Now < target)
		{
			gradual->Now = target;
		}
	}
}

