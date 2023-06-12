#ifndef __SPEED_CTRL_H
#define __SPEED_CTRL_H

#include "sys.h"
#include "stdbool.h"

struct Gradual
{
	float Last;			//改变时的上一次的值
	float Now;			//当前值
	float D_value;		//改变时刻的目标值与当前值的差
};

struct Motors
{
	float Lspeed,Rspeed;	//速度
	float Cspeed;					//寻迹速度
	float Gspeed;					//自平衡速度
	float GyroT_speedMax;	//转弯最大速度
	float GyroG_speedMax;	//自平衡最大速度
	
	int encoder_avg;	//编码器读数 
	float Distance;		//路程
	
	float Cincrement;	//循迹加速度
	float Gincrement;	//非循迹加速度  
	
	bool is_UP;
	bool is_DOWM;
};

extern volatile struct Motors motor_all;

extern struct Gradual TC_speed, TG_speed;

void gradual_cal(struct Gradual *gradual, float target, float increment);
void CarBrake(void);

#endif
