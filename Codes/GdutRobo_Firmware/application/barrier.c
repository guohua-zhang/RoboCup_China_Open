#include "barrier.h"
#include "sys.h"
#include "bsp_delay.h"
#include "speed_ctrl.h"
#include "bsp_motor.h"
#include "pid.h"
#include "bsp_imu.h"
#include "scaner.h"
#include "turn.h"
#include "map.h"
#include "tim_it.h"
#include "bsp_linefollower.h"
#include "pid.h"
#include "math.h"
#include "bsp_QRscan.h"
#include "QR_action.h"
#include "bsp_buzzer.h"
#include "scaner.h"
#include "bsp_led.h"

#define Up_pitch -15
#define Down_pitch 10
#define BACK_SPEED  -30
#define BACK_SPEED1 -10
#define GOSPEED 10

extern int BW_num[];
extern int value;

//防抖动系列函数
//1._shake在平台识别过程中，防止因一些意外因素导致车提前停止,同时完成撞板的目的
//2._Rshake用于判断其是否经过波浪板
//3._pshake识别处于下坡状态
void Anti_shake(int Uneed_time)
{
	uint32_t i = 0;  //循环变量设置成unit32_t避免for循环最后的奇怪指令
	for(i = 0; i < Uneed_time ; i++)
	{
		HAL_Delay(1);
		while(Infrared_ahead == 0);
	}
}

void Anti_encoder(int Uneed_time)
{
	uint32_t i = 0;  //循环变量设置成unit32_t避免for循环最后的奇怪指令
	for(i = 0; i < Uneed_time ; i++)
	{
		HAL_Delay(1);
		while(read_encoder(2) == 0);
	}
}

//void Anti_Rshake()
//{
//	uint32_t i = 0;
//	uint32_t cnt = 0;
//	while(i < 148)
//	{
//		for(i = 0; i < 150 ; i++)
//		{
//			HAL_Delay(1);
//			if(fabs(imu.roll) > 2) cnt++;
//			if(cnt == 30)
//			{
//				i = 0;
//				cnt = 0;
//				break;
//			}
//			//while(fabs(imu.roll) > 2);
//		}
//	}
//}

int Anti_Pshake()
{
	int sum = 0;
	for(uint32_t i = 0; i < 5; i++)
	{
		HAL_Delay(1);
		sum += imu.pitch;
	}
	if(sum/5 > 3) return 1;
	else return 0;
}

/************************************************/

void Stage()		//flag==1时取绝度角度，flag==0时取相对角度
{
	float num = 0; //distan = 0;
	
	pid_mode_switch(is_Line);
	
	if (nodesr.nowNode.nodenum == P2)
	{
		Stage_P2();
		return;
	}
	
	while(imu.pitch > -5); //上斜坡时认为上了平台
	angle.AngleG = getAngleZ();
	motor_all.Gspeed = 50;
	pid_mode_switch(is_Gyro);
	
	//while(Infrared_ahead == 0);		//撞挡板
	
	Anti_shake(200);  //撞挡板

	num = motor_all.Distance;				//前进一段距离
	while(motor_all.Distance - num < 10);
	
	//delay_ms(300);
	CarBrake();
	HAL_Delay(1000);
	mpuZreset(imu.yaw, nodesr.nowNode.angle);  //陀螺仪校正
	num = motor_all.Distance;				//后退一段距离
	pid_mode_switch(is_No);
	
//	motor_L0.target = motor_L1.target = BACK_SPEED;
//	motor_R0.target = motor_R1.target = BACK_SPEED;
	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
	
	while (num-motor_all.Distance < 8); //原来是后退6
	CarBrake();
	motor_pid_clear();
//	HAL_Delay(400);
	
	//myAction(); //机器人动作+寻宝检测 
	
	/*****************若扫不到二维码******************/
 	if((BW_num[0] == 0 && nodesr.nowNode.nodenum == P1)
	  ||(BW_num[1] == 0 && nodesr.nowNode.nodenum == P3)
	  ||(BW_num[1] == 0 && nodesr.nowNode.nodenum == P4)
	  ||(BW_num[2] == 0 && nodesr.nowNode.nodenum == P5)
	  ||(BW_num[2] == 0 && nodesr.nowNode.nodenum == P6)
	)
	{
		num = motor_all.Distance;				//后退一段距离
		motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED1;
		while (num-motor_all.Distance < 4);
		CarBrake();
		//motor_pid_clear();
		Turn_Angle_Relative(-30);
		while (fabs(angle.AngleT-getAngleZ())>2);
		Turn_Angle_Relative(30);
		while (fabs(angle.AngleT-getAngleZ())>2);
		motor_pid_clear();
	}
	/***************************************************/
	
	/**************动作****************/
	Shine_o_f();       //闪灯一次
	actions(5);     
	delay_ms(100);
	actions(3);
	delay_ms(100);
	Turn_Angle_Relative(179);
	while (fabs(angle.AngleT-getAngleZ())>2);
	
	if(check_BW(nodesr.nowNode.nodenum)) //检测宝物
		{
			actions(1);
			delay_ms(50);
			Turn_Angle360();
			actions(2);
//			delay_ms(50);
		}
	actions(6);
//	delay_ms(100);
	/**********************************/
	
	//Turn_Angle_Relative(179);		//转>=180度
	motor_pid_clear();
	motor_all.Cspeed=50;
	motor_all.Cincrement = 1.0;
//	line_pid_param.kp = 9;
	pid_mode_switch(is_Line);
	while (imu.pitch < Down_pitch);	//下坡时认为离开平台  
	nodesr.nowNode.function=0;	//清除障碍标志
	nodesr.flag|=0x04;	//到达路口
}


void Stage_P2()		//flag==1时取绝度角度，flag==0时取相对角度
{
	//float num = 0;
	while(imu.pitch > Up_pitch); //上斜坡时认为上了平台
	angle.AngleG=getAngleZ();
	motor_all.Gspeed=40;
	pid_mode_switch(is_Gyro);
	while (Infrared_ahead == 0);

	CarBrake();
	
	Turn_Angle_Relative(182);		//转>=180度，
	while (fabs(angle.AngleT - getAngleZ())>5);
	motor_pid_clear();
	nodesr.nowNode.function=0;	//清除障碍标志
	nodesr.flag|=0x04;	//到达路口
}

/***************************************************
过长桥

************************************************/

void Barrier_Bridge(float step,float speed)	//过长桥
{
	float num = 0;
	
	motor_all.Cspeed = 60;    //原来80   90
	pid_mode_switch(is_Line);
	while(imu.pitch > Up_pitch);	//还在平地	
	//is_Up = true;
	motor_all.Gspeed = 60;    //90 
	angle.AngleG = nodesr.nowNode.angle;
	pid_mode_switch(is_Gyro);
	
	struct PID_param origin_param = gyroG_pid_param;
	gyroG_pid_param.kp = 1.7;
	
	while(imu.pitch <= Up_pitch);	//上桥
	//is_Up = false;

	//motor_all.Gspeed = 110;   //原来65   90
	num = motor_all.Distance;
	while(imu.pitch < Down_pitch)          
	{
		if (infrared.inside_left == 1||infrared.outside_left == 1)
			angle.AngleG = nodesr.nowNode.angle - 8; 
		else if (infrared.inside_right == 1||infrared.outside_right == 1)
			angle.AngleG = nodesr.nowNode.angle + 8; 
		else
			angle.AngleG = nodesr.nowNode.angle;
		
		if (motor_all.Distance-num < 60)
			motor_all.Gspeed = 70;     //原来是80  90
		else
			motor_all.Gspeed = 40;      //原来是65   75
	}
	angle.AngleG = nodesr.nowNode.angle;
	delay_ms(500);
	motor_all.Cspeed = 90;
	pid_mode_switch(is_Line);
	nodesr.nowNode.function = 0;
	nodesr.flag |= 0X04;  //到达路口
}

//过楼梯
void Barrier_Hill(uint8_t order)  //楼梯数量
{
	struct PID_param origin_param = line_pid_param;
	motor_all.Cspeed = 50;   //原来是60
	line_pid_param.kp = 50;
	pid_mode_switch(is_Line);
	while ( imu.pitch > -10 );  //平地
	motor_all.Cspeed = 45;
	while(imu.pitch < -10);  //坡上
	while(imu.pitch<10);  //第一个平地
	
	if (order == 2)
	{
		while (imu.pitch > 10);  //下第一个坡
		while (imu.pitch > -10); //波谷
		while (imu.pitch < -10);  //第二个上坡
		while (imu.pitch < 10);  //第二个平地
	}
	line_pid_param = origin_param;
	nodesr.nowNode.function=0;//清除障碍标志
	nodesr.flag|=0x04;	//到达路口
}



void Sword_Mountain()
{
	float num;
	struct PID_param origin_param = line_pid_param;
	struct PID_param origin_param1 = gyroG_pid_param;
	num = motor_all.Distance;
	
	line_pid_param.kp = 50;
	
	motor_all.Cspeed = 20;   //原来30
	
	//mpuZreset(imu.yaw, nodesr.nowNode.angle);  //陀螺仪校正
	while(motor_all.Distance - num < 50);
	angle.AngleG = getAngleZ();
	
	gyroG_pid_param.kp = 1.4;   //拉大刀山的自平衡kp
	motor_all.Gspeed = 20;
	pid_mode_switch(is_Gyro);

	while(imu.pitch > -2);
	buzzer_on();
	
	//motor_all.Gspeed = 20;  //30
	
//	motor_all.Lspeed = 24;
//	motor_all.Rspeed = 20;
//	pid_mode_switch(is_No);
	
	//angle.AngleG = 0;      //此处不知道如何应用
	//pid_mode_switch(is_Gyro);
	while(imu.pitch < 2);
	gyroG_pid_param = origin_param1;
	line_pid_param = origin_param;
	buzzer_off();
	
	//while(Scaner.ledNum==0);
	 
	nodesr.nowNode.function=0;//清除障碍标志
	nodesr.flag|=0x04;	//到达路口
}

/*****************************************************************************
上珠峰，包含下珠峰
*****************************************************************************/
void Barrier_HighMountain()
{
	float num = 0;
	
	struct PID_param origin_param = line_pid_param;
	
	motor_all.Cspeed = 40;
	pid_mode_switch(is_Line);
	line_pid_param.kp = 25;
	line_pid_param.kd = 15;
	
	motor_all.Cspeed = 70; //最开始还没上坡的速度 90
	motor_all.Cincrement = 2;
	pid_mode_switch(is_Line);
	while(imu.pitch>-17);
    
	motor_all.Cincrement = 1.8; //上坡的速度  2
	motor_all.Cspeed = 80;//80
	
//	num=motor_all.Distance;
//	while(motor_all.Distance-num<55*69.17); //上坡的长度
//	angle.AngleG=getAngleZ();
//	PIDMode=GYRO;
//	motor_all.Gspeed=20;
//motor_all.Cspeed=25;
	
	while (imu.pitch < -17);   //上第一个坡
	pid_mode_switch(is_Gyro);
	motor_all.Gspeed = 80;
	while (imu.pitch > -17);   //在第二个平台
	pid_mode_switch(is_Line);
	motor_all.Cspeed = 80;
	while (imu.pitch < -17);   //上第二个坡
	pid_mode_switch(is_Gyro);
	motor_all.Gspeed = 40;
	
	Anti_shake(300);
//	while(Infrared_ahead == 0);		//撞挡板
//	delay_ms(300);	
	CarBrake();
	
	mpuZreset(imu.yaw, nodesr.nowNode.angle);  //陀螺仪校正
	
	num = motor_all.Distance;
	pid_mode_switch(is_No);
	
//	motor_L0.target = motor_L1.target = BACK_SPEED;
//	motor_R0.target = motor_R1.target = BACK_SPEED;
	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;

	
	while(num-motor_all.Distance < 4);
	CarBrake();
	
	motor_all.GyroT_speedMax = 60;   //降低转圈速度
  //Turn_Angle_Relative(181); //转动
		
	Shine_o_f();       //闪灯一次
	actions(5);     //景点动作
	delay_ms(100);
	actions(3);
//	delay_ms(100); 
	Turn_Angle_Relative(179);
	
	while(fabs(angle.AngleT - getAngleZ())>2); //判断误差
	if(check_BW(nodesr.nowNode.nodenum)) //检测宝物
		{
			actions(1);
			delay_ms(50);
			
			Turn_Angle_Relative(130);//160
			while(fabs(angle.AngleT - getAngleZ())>10);//4
			Turn_Angle_Relative(130);//160
			while(fabs(angle.AngleT - getAngleZ())>10);//4
			Turn_Angle_Relative(120);//120
			while(fabs(angle.AngleT - getAngleZ())>1);//4
			
			actions(2);
			delay_ms(50);
		}
//	actions(6);
//	delay_ms(100);
	
		
	motor_all.GyroT_speedMax = 80;   //恢复原来的转弯速度
	
	motor_pid_clear();
	delay_ms(100);

	pid_mode_switch(is_No);
	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
	HAL_Delay(1000);
	CarBrake();
	angle.AngleG = getAngleZ();
	
	Barrier_Down_HighMountain(60);

	
	line_pid_param = origin_param;
	nodesr.nowNode.function=0;//清除障碍标志
	nodesr.flag|=0x04;	//到达路口
}

/*****************************************************************************
函数功能：下珠峰，在转完180后调用
*****************************************************************************/
void Barrier_Down_HighMountain(float speed)
{
	
	buzzer_on();

	motor_all.Gspeed = 40;

	pid_mode_switch(is_Gyro);
	
	while(imu.pitch<20);
	actions(6);
	
	
	motor_all.Cspeed = speed;
	pid_mode_switch(is_Line);

	while(imu.pitch>5)
	{
		if (Scaner.ledNum >= 10)
		{
			angle.AngleG = getAngleZ();
			motor_all.Gspeed = speed;
			pid_mode_switch(is_Gyro);
			buzzer_off();
		}
	}
	
	while(imu.pitch<15);
	
	pid_mode_switch(is_Line);
	motor_all.Cspeed = speed;
	while(imu.pitch>=10);
	
}


void view()//打景点	
{	
	float num = 0;
	while(Infrared_ahead == 0);		//撞挡板
	delay_ms(200);
	num=motor_all.Distance;
	pid_mode_switch(is_No);
	
	motor_L0.target = motor_L1.target = BACK_SPEED;
	motor_R0.target = motor_R1.target = BACK_SPEED;
	
	while(num-motor_all.Distance<3);
	CarBrake();
	delay_ms(100);
	Turn_Angle_Relative(181);
	while(fabs(angle.AngleT-getAngleZ())>5);
	motor_pid_clear();
	motor_all.Cspeed=30;
	pid_mode_switch(is_Line);
	delay_ms(200);
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//到达路口
}

void view1()//打景点	
{	
	pid_mode_switch(is_Line);
	while(Infrared_ahead == 0);		//撞挡板
	delay_ms(100);
	//mpuZreset(gyro.yaw,nodesr.nowNode.angle-10);
	CarBrake();
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//到达路口
}

void back()
{
	pid_mode_switch(is_No);
	
	motor_L0.target = motor_L1.target = BACK_SPEED;
	motor_R0.target = motor_R1.target = BACK_SPEED;
	
	while(infrared.outside_left == 0 && infrared.outside_right == 0);
	CarBrake();
	//转绝对角度
	angle.AngleT = nodesr.nextNode.angle;
	pid_mode_switch(is_Turn);
	while(fabs(angle.AngleT - getAngleZ())>5);
	
	motor_pid_clear();
	pid_mode_switch(is_Line);
	//motor_all.Cspeed=15;
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//到达路口
}
/*****************************************************************************
过波浪板
速度   ，  波浪板长度（cm）
长度主要是防止误判提前进入波浪板，然后提前放下造成严重失误。
先走这一段长度，保证能进入波浪板，且避免提前误判成已经走去波浪板。
*****************************************************************************/
void Barrier_WavedPlate(float lenght)//波浪板长度
{
	float num = 0;
	//while(imu.pitch>-4);	
	//TIM8_PWM(2400,600,1500,1900);		//抬起寻迹板
//	Turn_Angle(need2turn(getAngleZ(),nodesr.nowNode.angle),36);
//	while(fabs(Turn_Angle_Targe-getAngleZ())>5);

	motor_all.Cspeed = 40;   
	pid_mode_switch(is_Line);
	num = motor_all.Distance;
	while( motor_all.Distance-num < lenght);	
	//TIM8_PWM(2400,600,1500,2500);
	//while(Scaner.detail==0);
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//到达路口

}


void South_Pole(float length)
{
	float num = 0;
	struct PID_param origin_param = line_pid_param;
	
	motor_all.Cspeed = 40;
	pid_mode_switch(is_Line);
	line_pid_param.kp = 20;
	line_pid_param.kd = 15;
	
	while(imu.pitch>-20);
	motor_all.Cspeed = 80;
	num = motor_all.Distance;
	while(motor_all.Distance - num < 60)
	
		angle.AngleG = getAngleZ();
		pid_mode_switch(is_Gyro);
		motor_all.Gspeed = 60;

	//while(imu.pitch<-10);
	
	/*while(Infrared_ahead == 0);
	delay_ms(300);*/
	Anti_shake(250);
	CarBrake();
	
	//myAction(); //机器人动作+寻宝检测
	
	mpuZreset(imu.yaw, nodesr.nowNode.angle);  //陀螺仪校正
	
	num = motor_all.Distance;
	pid_mode_switch(is_No);
	
//	motor_L0.target = motor_L1.target = BACK_SPEED;
//	motor_R0.target = motor_R1.target = BACK_SPEED;
	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;

	while(num - motor_all.Distance < 10);
	//Turn_Angle_Relative(182);
	CarBrake();
	
	Shine_o_f();       //闪灯一次
	actions(5);     //景点动作
	delay_ms(100);
	actions(3);
//	delay_ms(100); 
	Turn_Angle_Relative(179);
	while(fabs(angle.AngleT - getAngleZ())>5);
	
	if(check_BW(nodesr.nowNode.nodenum)) //检测宝物
		{
			actions(1);
			delay_ms(50);
			Turn_Angle360();
			actions(2);
			delay_ms(50);
		}
	actions(6);
	delay_ms(100);
		
	motor_pid_clear();
	
	motor_all.Cspeed = 65;
	pid_mode_switch(is_Line);
	while(imu.pitch<10);       //还在坡上
	while(imu.pitch>10);       //回到平地
	
	motor_all.Cspeed = 38;
	num = motor_all.Distance;
	while(motor_all.Distance-num < length);	
	
	line_pid_param = origin_param;  //恢复原来的PID参数
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//到达路口
}

#if 0
void QQB_1(void)
{
	float num = 0;
	CatchsensorNum=-3.5;			//改变循迹中心点，在判断路口函数改回来
	PIDMode=Catch;
	motor_all.Cspeed=20;
	while(Scaner.detail!=0);	//循迹板上跷跷板
	PIDMode=Free;					
	L0speed=20;L1speed=20;R0speed=20;R1speed=20;
	while(imu.pitch>-15);		//检测到大角度，车子上跷跷板
	delay_ms(500);
	num=motor_all.Distance;
	PIDMode=GYRO;				//等车子走上跷跷板再走自平衡
	angle.AngleG=getAngleZ();
	motor_all.Gspeed=15;
	while(motor_all.Distance-num<700);			//读编码器走固定距离
	CarBrake();					//停止
	while(imu.pitch<15);		//等待跷跷板向下
	delay_ms(600);				//等待跷跷板停下
	PIDMode=Free;
	L0speed=5;L1speed=5;R0speed=30;R1speed=30;
	delay_ms(200);
	CarBrake();	
	if(Scaner.detail==0)
	{
		Turn_Angle(30,30);			//逆时针转40°
		while(fabs(Turn_Angle_Targe-getAngleZ())>5);
	}
	PIDMode=Catch;				//继续循迹
	motor_all.Cspeed=20;
	nodesr.nowNode.function=0;
}
#endif

/*void QQB_1(void)
{
	float num;
	struct PID_param origin_param = line_pid_param;
	
	num = motor_all.Distance;
	
	motor_pid_clear();
	pid_mode_switch(is_Line);
	motor_all.Cspeed = 30;  //给低速进入寻线 60
	
	scaner_set.CatchsensorNum = line_weight[7];   //给予左边权值
	line_pid_param.kp = 45;    //拉大KP 45
	
	
	if(nodesr.nowNode.nodenum == B9)
	{	
		while(fabs(motor_all.Distance - num) < 28);//适当调整
		pid_mode_switch(is_No);
		angle.AngleT = -92;//-89
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);
		
		angle.AngleG = -92;//-89
		pid_mode_switch(is_Gyro);
		motor_all.Gspeed = 20;
		while(imu.pitch > Up_pitch);//还在平地
		motor_all.Gspeed = 20;
		while(imu.pitch <= Up_pitch );
		
//		//start开环
//		while(imu.pitch > -1);//等待上坡
//		buzzer_on();
//		HAL_Delay(200);
//		buzzer_off();
//		motor_all.Lspeed = 20;
//		motor_all.Rspeed = 25; 
//		//end闭环
		
		while(imu.pitch <= -3);
		CarBrake();
		while(imu.pitch <= Down_pitch);
		angle.AngleT = -25;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);
		
	}
	else if(nodesr.nowNode.nodenum == B8)
	{	
		while(fabs(motor_all.Distance - num) <32);
		pid_mode_switch(is_No);
		angle.AngleT = 90;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);
		
		angle.AngleG = 90;
		pid_mode_switch(is_Gyro);
		motor_all.Gspeed = 30;
		while(imu.pitch > Up_pitch);//还在平地
		motor_all.Gspeed = 30;
		while(imu.pitch <= Up_pitch );
		
//		//start开环
//		while(imu.pitch > -1);//等待上坡
//		motor_all.Lspeed = 18;
//		motor_all.Rspeed = 25; 
//		while(imu.pitch < -16);
//		//end闭环
		
		CarBrake();
		angle.AngleT = -25;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);
	}
	
	
//	pid_mode_switch(is_Line);
//	motor_all.Cspeed = 20;
	scaner_set.CatchsensorNum = 0;
	line_pid_param = origin_param;  //恢复原来的PID参数
	nodesr.nowNode.function=0;
	nodesr.flag|=0X04;	//到达路口
	
}*/

void QQB_1(void)
{
	float num;
	struct PID_param origin_param = line_pid_param;
	
	num = motor_all.Distance;
	
	motor_pid_clear();
	pid_mode_switch(is_Line);
	motor_all.Cspeed = 60;  //给低速进入寻线
	
	scaner_set.CatchsensorNum = line_weight[5];    //给予左边权值
	line_pid_param.kp = 45;    //拉大KP 
	
	if(nodesr.nowNode.nodenum == B9)
	{	
		while(infrared.outside_right == 0);
		CarBrake();
		
		buzzer_on();
		
		//转去跷跷板的正向
		angle.AngleT = -89;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);
		/*angle.AngleT = 89;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);*/
		
		angle.AngleG = -89;
		pid_mode_switch(is_Gyro);
		motor_all.Gspeed = 20;
		while(imu.pitch > Up_pitch);//还在平地
		while(imu.pitch <= Up_pitch )
		{
			buzzer_off();
			if (infrared.outside_left == 1)
				angle.AngleG -= 4; 
			else if (infrared.outside_right == 1)
				angle.AngleG += 4; 
			else
				angle.AngleG = nodesr.nowNode.angle;
		}
		while(imu.pitch <= -4);
		buzzer_off();
		CarBrake();
		angle.AngleT = -70;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);
		//while(imu.pitch <= Down_pitch);
		
		/*angle.AngleT = 25;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);*/
		
	}
	else if(nodesr.nowNode.nodenum == B8)
	{	
		while(fabs(motor_all.Distance - num) <68);
		angle.AngleT = 90;
		pid_mode_switch(is_Turn);
		while(fabs(angle.AngleT - getAngleZ())>5);
		angle.AngleG = 90;
		pid_mode_switch(is_Gyro);
		motor_all.Gspeed = 30;
		while(imu.pitch > Up_pitch);//还在平地
		motor_all.Gspeed = 30;
		while(imu.pitch <= Up_pitch );
		CarBrake();
//		angle.AngleT = -25;
//		pid_mode_switch(is_Turn);
//		while(fabs(angle.AngleT - getAngleZ())>5);
	}
	
	
//	pid_mode_switch(is_Line);
	pid_mode_switch(is_Gyro);
	num = motor_all.Distance;
	motor_all.Gspeed = 10;
	while(fabs(motor_all.Distance - num) < 5);
	scaner_set.CatchsensorNum = 0;
	line_pid_param = origin_param;  //恢复原来的PID参数
	pid_mode_switch(is_Line);
	nodesr.nowNode.function=0;
	nodesr.flag|=0X04;	//到达路口
	
}

void door()
{
	float num = 0;
	//	buzzer_on();
	static u8 flag=0;
	motor_all.Cspeed = 60;
	pid_mode_switch(is_Line);
	if(flag==2)	
	{
		route_reset(flag);
		flag = 3;
		return;
	}
	else if(flag==4)
	{
		route_reset(flag);
		return;
	}
	while(1)
	{		//走一米来检测全白停止线
		if(Scaner.ledNum>=8)
		{
			buzzer_on();
			CarBrake();
			HAL_Delay(400);    //根据实际情况做时间延迟修改
			if(value == 1)
			{
				buzzer_off();
				if(flag==0)	//第一个路口的交通指示牌是红牌，还不能把路线连接起来
				{
					map.point -= 2;
					route[map.point] = N8;
					pid_mode_switch(is_No);			//后退
	
					//motor_L0.target = motor_L1.target = BACK_SPEED;
					//motor_R0.target = motor_R1.target = BACK_SPEED;
					motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
	
					num = motor_all.Distance;
					while(num-motor_all.Distance<20);
					Turn_Angle_Relative(181);	//	转到当前结点方向
					while(fabs(angle.AngleT-getAngleZ())>3);
					nodesr.nowNode	= Node[getNextConnectNode(N10,N3)];		//重新设置nowNode
					nodesr.nowNode.step = 55;
					nodesr.nowNode.flag|=STOPTURN;
					nodesr.nowNode.speed=90;
					pid_mode_switch(is_Line);
					nodesr.flag|=0x20;
					flag=1;
					return ;
				}
				else if(flag==1)//第二个也是红色，还不能把路线连接起来
				{		
					map.point-=2;		
					route[map.point]=N4;
					pid_mode_switch(is_No);			//后退

					motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;

					num=motor_all.Distance;
					while(num-motor_all.Distance<20);
					Turn_Angle_Relative(181);	//	转到当前结点方向
					while(fabs(angle.AngleT-getAngleZ())>3);
					nodesr.nowNode	= Node[getNextConnectNode(N8,N3)];		//重新设置nowNode
					nodesr.nowNode.step = 35;
					nodesr.nowNode.speed = 90;
					nodesr.nowNode.flag|=STOPTURN;
					pid_mode_switch(is_Line);
					nodesr.flag|=0x20;				
					flag=2;
					return ;
				}		
				else if(flag==3)
				{
					map.point-=2;		
					route[map.point]=N8;
					pid_mode_switch(is_No);	 //后退
	
					//motor_L0.target = motor_L1.target = BACK_SPEED;
					//motor_R0.target = motor_R1.target = BACK_SPEED;
					motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
	
					num=motor_all.Distance;
					while(num-motor_all.Distance<20);
					Turn_Angle_Relative(181);	//	转到当前结点方向
					while(fabs(angle.AngleT-getAngleZ())>3);
					nodesr.nowNode	= Node[getNextConnectNode(N12,N5)];		//重新设置nowNode
					nodesr.nowNode.step=55;
					nodesr.nowNode.speed=100;
					nodesr.nowNode.flag|=STOPTURN;
					pid_mode_switch(is_Line);
					nodesr.flag|=0x20;				
					flag=4;
					return ;
				}
			}	
			else
			{
				// buzzer_off();
				motor_all.Cspeed=60;
				pid_mode_switch(is_Line);
				nodesr.nowNode.function=1;
				HAL_Delay(1000);
				break;
			}
		}
		else 
		{
			motor_all.Cspeed=60;
			pid_mode_switch(is_Line);
		}				
	}

	if(flag != 3)   route_reset(flag);//路线重新设置

}


void route_reset(u8 flag)
{
	u8 temp=0,i=0;
	temp = map.point - 1;
	if(flag==0)//第一个门开着
	{			
		while(1)
		{	
			route[temp++]=door1route[i++];		//路线连接
			if(door1route[i]==255)
			{
				route[temp]=door1route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N3,N10)];		//重新设置nowNode
				nodesr.nextNode = Node[getNextConnectNode(N10,route[map.point-1])];	//重新设置nextNode	
				nodesr.nowNode.step=10;
				nodesr.nowNode.speed=70;
				nodesr.nowNode.function=1;
				break;
			}
		}
	}
	else if(flag==1)//第二个门开着
	{		
		while(1)
		{	
			route[temp++]=door2route[i++];	//路线连接
			if(door2route[i]==255)
			{
				route[temp]=door2route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N3,N8)];		//重新设置nowNode
				nodesr.nextNode = Node[getNextConnectNode(N8,route[map.point-1])];	//重新设置nextNode	
				nodesr.nowNode.step=30;
				nodesr.nowNode.speed=70;
				nodesr.nowNode.function=1;
				break;
			}
		}
	}	
	else if(flag==2)//去到第四个通道口
	{	
		while(1)
		{	
			route[temp++]=door3route[i++];	//路线连接
			if(door3route[i]==255)
			{
				route[temp]=door3route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N3,N4)];		//重新设置nowNode
				nodesr.nextNode = Node[getNextConnectNode(N4,route[map.point-1])];	//重新设置nextNode
				nodesr.nowNode.step=10;
				nodesr.nowNode.speed=70;
				nodesr.nowNode.function=1;
				break;
			}
		}
	}
	else if(flag==4)//第三个门开着
	{
		while(1)
		{	
			route[temp++]=door4route[i++];	//路线连接
			if(door4route[i]==255)
			{
				route[temp]=door4route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N5,N8)];		//重新设置nowNode
				nodesr.nextNode = Node[getNextConnectNode(N8,route[map.point-1])];	//重新设置nextNode
				nodesr.nowNode.step=10;
				nodesr.nowNode.speed=60;
				nodesr.nowNode.function=1;
				break;
			}
		}
	}
}

void undermou(void)
{
	float num = 0;
	while(imu.pitch > -3);
	motor_all.Cspeed = 40;
	num = motor_all.Distance;
	while(motor_all.Distance - num < 50);
	buzzer_on();
	if(nodesr.nowNode.nodenum == N14)
	{
		motor_all.Cspeed = 100;
		num = motor_all.Distance;
		while(motor_all.Distance - num < 105);
		motor_all.Cspeed = 30;
	}
	while(!deal_arrive());
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//到达路口
}
void S_curve(void)
{
	float num;
	struct PID_param origin_param = line_pid_param;
	
	num = motor_all.Distance;
	motor_all.Cspeed = nodesr.nowNode.speed;
	
	if (nodesr.nextNode.nodenum == N13)
	{
		motor_all.Cspeed = 110;
		while (fabsf(motor_all.Distance-num) < 110);
		motor_all.Cspeed = 60;
		scaner_set.CatchsensorNum = line_weight[11];  //C1->C2
		line_pid_param.kp = 40;
		while (fabsf(getAngleZ() - nodesr.nextNode.angle) > 10);
	}
	else if (nodesr.nextNode.nodenum == C1)
	{
		motor_all.Cspeed = 110;
		while (fabsf(motor_all.Distance-num) < 60);
		scaner_set.CatchsensorNum = line_weight[5];
		line_pid_param.kp = 70;
		while (fabsf(getAngleZ() - nodesr.nextNode.angle) > 10);
	}
	
	line_pid_param = origin_param;
	scaner_set.CatchsensorNum = 0;
	
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//到达路口
}


void ignore_node(void)
{
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//到达路口
}




