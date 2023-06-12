#include "turn.h"
#include "bsp_imu.h"
#include "pid.h"
#include "speed_ctrl.h"
#include "math.h"
#include "map.h"
#include "tim_it.h"

volatile struct Angle angle = {0, 0};

//输出当前角度与目标角度的最小夹角
float need2turn(float nowangle,float targetangle)
{			
	float need2Turn;		

	need2Turn=targetangle-nowangle;		//实际所需转的角度
	if(need2Turn>180)	need2Turn -= 360;
  else if(need2Turn<-180)	need2Turn += 360;
	
  return need2Turn;		
}

//陀螺仪软校准  
void mpuZreset(float sensorangle ,float referangle)
{	
	imu.compensateZ=need2turn(sensorangle,referangle);
}


//获取Z角度
float getAngleZ(void) 
{
	float targetangle;
	targetangle = imu.yaw + imu.compensateZ;

	if(targetangle>180)	targetangle -= 360;
	else if(targetangle<-180)	targetangle += 360;
	
	return targetangle;
}


/*****************************************************************************
函数编写人及更新日期：陈梓华，7-19
函数功能：陀螺仪Z轴转角度，转相对角度
输入：要转的角度 ，速度
输出：无
*****************************************************************************/
void Turn_Angle_Relative(float Angle1)//左180到右-180,速度必须是正的，
{
	float Turn_Angle_Before = 0, Turn_Angle_Targe = 0;
	
	Turn_Angle_Before = getAngleZ();//读取当前的角度//@@@@@
	Turn_Angle_Targe = Turn_Angle_Before+Angle1;//目标角度设为绝对坐标
	/*******************如果存在临界状态，把目标角度转化为绝对坐标******180 0 -180*************/
    if(Turn_Angle_Targe>180){				
    	Turn_Angle_Targe = Turn_Angle_Targe-360;
    }
    else if(Turn_Angle_Targe<-180){
    	Turn_Angle_Targe=Turn_Angle_Targe+360;
    }
	
	angle.AngleT = Turn_Angle_Targe;
	pid_mode_switch(is_Turn);  //进入转弯
}


/*****************************************************************************
函数功能：陀螺仪Z轴结合PID原地转角度
					填要转到的角度(绝对角度)
					用法：Target = ?,PIDMode = is_Turn,while(fabs(getAngleZ()-Target)<1);
*****************************************************************************/
uint8_t Turn_Angle(float Angle)	
{
	float GTspeed, now_angle;
	
	//临界处理
	if(Angle>180)	Angle -= 360;
	else if(Angle<-180)	Angle += 360;
	
	now_angle = getAngleZ();
	if (fabsf(Angle-now_angle) < 1)
	{
		motor_all.Lspeed = motor_all.Rspeed = 0;
		gyroT_pid.integral = 0;
		gyroT_pid.output = 0;
		return 1;
	}
	
	gyroT_pid.measure = need2turn(now_angle, Angle);
	gyroT_pid.target = 0;

	GTspeed = positional_PID(&gyroT_pid, &gyroT_pid_param);
	
	if(GTspeed >= motor_all.GyroT_speedMax) 
		GTspeed = motor_all.GyroT_speedMax;
	else if(GTspeed <= -motor_all.GyroT_speedMax) 
		GTspeed = -motor_all.GyroT_speedMax;

	motor_all.Lspeed = GTspeed; 
	motor_all.Rspeed = -GTspeed;
	
	return 0;
}

/*****************************************************************************
函数功能：原地旋转360度
		说明：运用多次旋转，设定阈值，到达目标值前进行更新
*****************************************************************************/
void Turn_Angle360(void)
{
	/*if(nodesr.nowNode.nodenum == P7||nodesr.nowNode.nodenum == P8)
	{
		Turn_Angle_Relative(130);//160
		while(fabs(angle.AngleT - getAngleZ())>10);//4
		Turn_Angle_Relative(130);//160
		while(fabs(angle.AngleT - getAngleZ())>10);//4
		Turn_Angle_Relative(120);//120
		while(fabs(angle.AngleT - getAngleZ())>2);//4
	}*/
	
		Turn_Angle_Relative(130);//160
		while(fabs(angle.AngleT - getAngleZ())>10);//4
		Turn_Angle_Relative(130);//160
		while(fabs(angle.AngleT - getAngleZ())>10);//4
		Turn_Angle_Relative( 120);//120
		while(fabs(angle.AngleT - getAngleZ())>1);//4

}

/*****************************************************************************
函数功能：陀螺仪Z轴自平衡行走
*****************************************************************************/
uint8_t runWithAngle(float angle_want,float speed)
{
	float GGspeed, now_angle;
	
	now_angle = getAngleZ();
	
//	if (fabsf(angle_want-now_angle) < 1)
//	{
//		motor_all.Lspeed = motor_all.Rspeed = speed;
//		return 0;
//	}
	
	gyroG_pid.measure = need2turn(now_angle, angle_want);  
	gyroG_pid.target = 0;
	
	GGspeed = positional_PID(&gyroG_pid, &gyroG_pid_param);
	
	if(GGspeed >= motor_all.GyroG_speedMax) 
		GGspeed = motor_all.GyroG_speedMax;
	else if(GGspeed <= -motor_all.GyroG_speedMax) 
		GGspeed = -motor_all.GyroG_speedMax;

	motor_all.Lspeed = speed+GGspeed*speed/50;
	motor_all.Rspeed = speed-GGspeed*speed/50;	
	
	return 1;
}

/*****************************************************************************
函数功能：差速转（最好还是用上pid吧）
*****************************************************************************/
void AdCircle(float speed, float radius) 
{
	motor_all.Lspeed = speed - radius;
	motor_all.Rspeed = speed + radius;	
}





