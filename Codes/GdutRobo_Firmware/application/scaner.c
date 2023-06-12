#include "scaner.h"
#include "map.h"
#include "math.h"
#include "turn.h"
#include "stdio.h"
#include "pid.h"
#include "speed_ctrl.h"

#define LINE_SPEED_MAX 50

#define Speed_Compensate

const float line_weight[16] = {-3.1,-2.4,-1.8,-1.3,-0.9,-0.6,-0.4,-0.3,0.3,0.4,0.6,0.9,1.3,1.8,2.4,3.1};//0.3

struct Scaner_Set scaner_set = {0,0};

//float CatchsensorNum = 0;	//寻迹时的目标传感器位置，一般情况下为0，微微会用到
SCANER Scaner;
/*****************************************************************************
函数编写人及更新日期：陈梓华07.16
函数功能：寻迹，在定时器之中运行，利用PIDMode标志位开启关闭
输入：寻迹速度
输出：无
*****************************************************************************/
void Go_Line(float speed)
{								//寻迹，入口变量寻迹速度
	float Fspeed;				//经过PID运算后的结果
					
	line_pid_obj.measure = Scaner.pow;  //当前循迹板所在的位置，从左到右-7到0到0到7
	line_pid_obj.target = scaner_set.CatchsensorNum;
	
	Fspeed = positional_PID(&line_pid_obj, &line_pid_param); //进行位置PID运算
	
	if (Fspeed>=LINE_SPEED_MAX) 
		Fspeed = LINE_SPEED_MAX;
	else if (Fspeed<=-LINE_SPEED_MAX)
		Fspeed = -LINE_SPEED_MAX;
	
	Fspeed *= fabsf(speed)/50;
	
	motor_all.Lspeed = speed-Fspeed;
	motor_all.Rspeed = speed+Fspeed;
}


void powCal(SCANER *scaner, unsigned char sensorNum, int8_t edge_ignore) //计算循迹版亮灯的位置
{
	float _pow = 0;
	u8 lednum=0;
	u8 linenum=0;
	int8_t lednum_tmp = 0;
	int8_t i=0;

	for(i=0; i<sensorNum; i++) 		//从小车方向从左往右数亮灯数和引导线数
	{															//linenum用来记录有多少条线，line用来记录第几条线。
		if((scaner->detail&(1<<i)))
		{
			lednum++;
			if(!(scaner->detail&(1<<(i+1)))) 
				++linenum;			//先读取亮灯数和引导线数，检测到从1变为0认为一条线
		}
	}
	scaner->lineNum = linenum;
	scaner->ledNum=lednum;
	
	if ((nodesr.nowNode.flag & LEFT_LINE) == LEFT_LINE)  //左循线
	{
		for (i=0; i<sensorNum; i++)
		{
			lednum_tmp += (scaner->detail>>i)&0X01;
			_pow += ((scaner->detail>>i)&0X01) * line_weight[i];
			if ((scaner->detail>>i) & 0X01)
				if (!((scaner->detail>>(i+1))&0x01))
					break;
		}
		//printf("%f\r\n",_pow);
	}
	else if ((nodesr.nowNode.flag & RIGHT_LINE) == RIGHT_LINE)  //右循线
	{
		for (i=sensorNum-1; i>=0; i--)
		{
			lednum_tmp += (scaner->detail>>i)&0X01;
			_pow += ((scaner->detail>>i)&0X01) * line_weight[i];
			if ((scaner->detail>>i) & 0X01)
				if (!((scaner->detail>>(i-1))&0x01))
					break;
		}
		//printf("%f\r\n",_pow);
	}
	else//正常寻线
	{
		if (lednum <= 7)  ////防止过多的灯带来的干扰  7
		{
			if (lednum >=4 )
				edge_ignore = 3;//原来是3

//			//******防止直线冲的路线边缘线的影响
//			if(nodesr.nowNode.nodenum == N3 || nodesr.nowNode.nodenum == N6 || nodesr.nowNode.nodenum == N5)
//			{
//				edge_ignore = 5;
//			}
//			//**********
			
			for(i=edge_ignore; i<sensorNum-edge_ignore; i++) 
			{
				lednum_tmp += (scaner->detail>>i)&0X01;
				_pow += ((scaner->detail>>i)&0X01) * line_weight[i];
			}	
			//printf("%f\r\n",_pow);
		}
		else
			return;
	}
	
	if(lednum==0)
	{
		_pow=0;
	}	
	else
	{
		_pow/=(float)lednum_tmp;		//取平均
	}
	
	scaner->pow = _pow;
}




