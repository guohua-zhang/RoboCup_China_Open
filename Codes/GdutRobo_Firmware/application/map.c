#include "map.h"
#include "barrier.h"
#include "sys.h"
#include "usart.h"
#include "bsp_delay.h"
#include "scaner.h"
#include "bsp_imu.h"
#include "turn.h"
#include "speed_ctrl.h"
#include "tim_it.h"
#include "bsp_linefollower.h"
#include "math.h"
#include "bsp_buzzer.h"

struct Map_State map = {0,0};

//u8 route[100] = {C8, C4, N20, P7, N20, 0XFF};

//u8 route[100] = {C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,0XFF};

//u8 route[100] = {N9,B9,N7,P5,N7,0XFF};

//u8 route [100] = {B1,N1,P1,N1,B2,N4,N5,N6,P4,N6,N5,N4,N3,P3,N3,N10,0XFF};    //走门路线

//u8 route[100] = {P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,0XFF};   //qqb

//u8 route[100] = {B6,N20,P7,N20,0XFF};

//u8 route[100] = {N6,N5,N4,N3,P3,N3,N10,0XFF};

u8 route[100] = {P8,C9,N22,B6,N20,P7,N20,0XFF};


//正常路线
//u8 route[100]={B1,N1,P1,N1,B2,N4,N3,P3,N3,
//				N4,N5,
//				N6,P4,N6,C1,C2,N13,P6,N13,N19,C6,B7,N22,C9,P8,
//				C9,N22,B6,N20,P7,
//				N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,/*B8,N9,N10,
//				N12,N13,C2,C1,N6,N5,N4,B3,N2,P2,*/0XFF};
				
u8 door1route[100]={N15,C5,B4,N18,N16,N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,
				    C9,P8,C9,N22,B6,N20,P7,
					N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,0XFF};

					
u8 door2route[100]={N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,P8,
					C9,N22,B6,N20,P7,
					N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,0XFF};				

u8 door3route[100]={N5,N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,P8,
					C9,N22,B6,N20,P7,
					N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,0XFF};

u8 door4route[100]={N12,N13,P6,N13,N18,B5,C6,B7,N22,C9,P8,
					C9,N22,B6,N20,P7,
					N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,0XFF};

	
NODESR nodesr;	//运作中间变量
					

//地图初始化
void mapInit()
{
	u8 i=0;
	nodesr.nowNode.nodenum = C9;		//起始点   //N2
	nodesr.nowNode.angle = 89;		//起始角度   //0
	nodesr.nowNode.function = 1;	//起始函数   //1
	nodesr.nowNode.speed = 40;//60             //80
	nodesr.nowNode.step= 2;//30               //20
	nodesr.nowNode.flag = DLEFT|CLEFT|STOPTURN;    //CLEFT|RIGHT_LINE
//	for(i=0;i<107;i++)				//把地图长度信息的厘米转化成编码器数
//		Node[i].step*=58.22;
//	for(i=0;i<107;i++)				//把地图长度信息的厘米转化成编码器数
//		Node[i].flag|=STOPTURN;
	for(i=0;i<118;i++)			//全地图速度调整
		Node[i].speed*=1.5;	
}


u8 getNextConnectNode(u8 nownode,u8 nextnode) 
{//得到本节点到相邻结点的地址
	unsigned char rest = ConnectionNum[nownode];
	unsigned char addr = Address[nownode];
	int i = 0;
	for (i = 0; i < rest; i++) 
	{
		if(Node[addr].nodenum == nextnode)
		{			
			return addr;
		}
		addr++;
	}
	return 0;
}

u8 deal_arrive()
{				
	register uint8_t lnum = 0, i = 0;
	register uint16_t seed = 0;
	static uint16_t mul2sing = 0, sing2mul = 0;
	
	if ((nodesr.nowNode.flag & DLEFT) == DLEFT)  //左半边
	{
		//左边6个灯任意5个亮即可
		if (Scaner.ledNum >= 5)
		{
			seed = 0X0001;
			for (i = 0; i<6; i++)
			{
				if (Scaner.detail & seed)
					++lnum;
				if (lnum >= 5)
					return 1;
				seed <<= 1;
			}
			lnum = 0;
		}
	}
	if ((nodesr.nowNode.flag & DRIGHT) == DRIGHT)//右半边
	{
		if (Scaner.ledNum>=5)
		{
			seed = 0X8000;
			for (i = 0; i<6; i++)
			{
				if (Scaner.detail & seed)
					++lnum;
				if (lnum >= 5)
					return 1;
				seed >>= 1;
			}
			lnum = 0;
		}
	}

	if ((nodesr.nowNode.flag & CLEFT) == CLEFT)//左分岔路
	{
		//左边数起第二、第三个灯任意一个亮即可
		 if( (Scaner.ledNum>=4&&Scaner.ledNum<=7) && (Scaner.detail&0x0006) )
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & MCLEFT) == MCLEFT)//左分岔路
	{
		//左边数起第一个灯亮即可
		 if( (Scaner.ledNum>=4&&Scaner.ledNum<=7) && (Scaner.detail&0x0001) )
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & MCRIGHT) == MCRIGHT)//左分岔路
	{
		//左边数起第一个灯亮即可
		 if( (Scaner.ledNum>=4&&Scaner.ledNum<=7) && (Scaner.detail&0x8000) )
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & CRIGHT) == CRIGHT)//右分岔路
	{
		 if( (Scaner.ledNum>=4&&Scaner.ledNum<=7) && (Scaner.detail&0x6000) )
		{
			return 1;
		}
	}
	
	else if ((nodesr.nowNode.flag & AWHITE) == AWHITE)//全白
	{
		 if((Scaner.ledNum>=10&&(Scaner.detail&0x1FF8)==0x1FF8))
		{
			return 1;
		}
	}
	
	else if ((nodesr.nowNode.flag & MUL2SING) == MUL2SING)//三分岔路
	{
//		 if(((Scaner.ledNum>=4&&Scaner.ledNum<=8)&&(Scaner.detail&0x07E0)==0x07E0))//三线
//		{
//			return 1;
//		}
		if (Scaner.lineNum > 1 || Scaner.ledNum >= 5)
			++mul2sing;
		
		if (mul2sing>=3 && Scaner.lineNum==1 && Scaner.ledNum<=3) //线数目由多变成一条
		{
			mul2sing = sing2mul = 0;
			return 1;
		}
	}
	
	else if ((nodesr.nowNode.flag & MUL2MUL) == MUL2MUL)  //线数目由多条变多条
	{
		if (Scaner.lineNum>1 || Scaner.ledNum>=5)
			++mul2sing;
		if (mul2sing>=3 && Scaner.lineNum==1)
			++sing2mul;
		if (sing2mul>=3 && (Scaner.lineNum>1 || Scaner.ledNum>=5))
		{
			mul2sing = sing2mul = 0;
			return 1;
		}
	}
	
	return 0;
}

void Cross()
{			
	float num = 0;
	static u8 _flag=1;			//结点动作标志位
	float turn;
	float turnspeed;
	static u8 cnt = 0;
	if(map.point == 0)
	{
		nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];//获取下一结点
	}
	if(_flag==1)//循迹
	{
		
		if(fabsf(motor_all.Distance) >= 0.8*nodesr.nowNode.step) 
		{
			_flag=0;
			if(((nodesr.nowNode.flag&RESTMPUZ)==RESTMPUZ))
			{
				if ((Scaner.detail & 0X0180) == 0X0180)
				mpuZreset(imu.yaw, nodesr.nowNode.angle);     //获取补偿角Z
				//nodesr.nowNode.flag&=~0x80;
			}
			
			//跟着霆哥删的
			/*if(((nodesr.nowNode.flag&RESTPID)==RESTPID))
			{
				//motor_pid_clear();
				nodesr.nowNode.flag&=~0x80;    //10000000
			}*/
			
			
			//条件1：转弯角度小于10度
			//条件2：下一个路口不是T字型路口
			if( (fabs(need2turn(getAngleZ(),nodesr.nextNode.angle))<=10) || (fabs(need2turn(nodesr.nowNode.angle,nodesr.nextNode.angle))<=10))
			{													 		
				motor_all.Cincrement = 2;	  //原来是1
				motor_all.Cspeed = 0.8*nodesr.nowNode.speed;
				
			}
			else
			{
				motor_all.Cincrement = 2.5;	//默认加速度 原来是1.5
				if ((nodesr.nowNode.flag & SLOWDOWN) == SLOWDOWN)
				{
					motor_all.Cspeed = 30;
				}
				if(0.5*nodesr.nowNode.speed>40)
				{
					motor_all.Cspeed=40;
				}
				else
				{
					motor_all.Cspeed=0.5*nodesr.nowNode.speed;
				}
			}
		}
		else
		{
			motor_all.Cincrement = 2.5;	//默认加速度  原来是1.5
			motor_all.Cspeed=0.8*nodesr.nowNode.speed;
		}
	}	
	else if(_flag==0)//判断路口
	{
			map_function(nodesr.nowNode.function);
		
		if(deal_arrive())	//判断是否到达路口
		{
			scaner_set.CatchsensorNum = 0;
			nodesr.flag |= 0x04;	//到达路口
			
			buzzer_on();
			if(nodesr.nowNode.nodenum == N8)
			{
				delay_ms(50);
			}
			if(nodesr.nowNode.nodenum == N13)
			{
				delay_ms(200);
			}
			else
			{
				delay_ms(100);
			}
			buzzer_off();
			
		}	
	}
	if((nodesr.flag&0x04)==0x04)//转弯
	{	
		nodesr.flag&=~0x04;		//	清除到达路口标志
		if(route[map.point-1] != 255)	 //route[i]==255代表一整条路线走完
		{		  
		        
			if ((fabs(need2turn(getAngleZ(),nodesr.nextNode.angle))<10) || (fabs(need2turn(nodesr.nowNode.angle,nodesr.nextNode.angle))<10))//如果下一结点角度与当前结点角度相同，不需要转，不需要减速	
			{													 		
				_flag=1;
			}
			else 
			{	                
				if(nodesr.nowNode.flag&STOPTURN)    //STOPTURN标志位待变原地转
				{      
					num = motor_all.Distance;
					if(nodesr.nowNode.nodenum == N22 || nodesr.nowNode.nodenum == C6 )   //C9点位我也做了判断
					{
						while(motor_all.Distance-num < 8);  //原来是16
					}
					else if(nodesr.nowNode.nodenum == C9)
					{
						while(motor_all.Distance-num < 12);
					}
					else if(nodesr.nextNode.nodenum == B6)
					{
						while(motor_all.Distance-num < 12);
					}
					else if(nodesr.nowNode.nodenum == N8)
					{
						while(motor_all.Distance-num < 8); 
					}
					else if(nodesr.nextNode.nodenum == N4)
					{
						while(motor_all.Distance-num < 27); 
					}
					else if(nodesr.nowNode.nodenum == N3)
					{
						while(motor_all.Distance-num < 15); 
					}
					else if(nodesr.nowNode.nodenum == C1)
					{
						while(motor_all.Distance-num < 22);
					}
					else
					{
						while(motor_all.Distance-num<10);	//步长延时，用于等待车中中心到达路口,原来是15
					}
					angle.AngleT = nodesr.nextNode.angle;  //转绝对角度
					pid_mode_switch(is_Turn);	
					while(fabs(nodesr.nextNode.angle-getAngleZ())>2);
					CarBrake();
					delay_ms(20);
					motor_pid_clear();
					_flag=1;

					/***霆哥*********/
//					Turn_Angle_Relative(need2turn(nodesr.nowNode.angle, nodesr.nextNode.angle));	
//					while(fabs(angle.AngleT - getAngleZ())>3);
//					CarBrake();
//					delay_ms(10);
//					motor_pid_clear();
//					_flag=1;				
					/****************/
					
					
				}
				else		//差速转
				{	
					if(fabs(need2turn(getAngleZ(),nodesr.nextNode.angle))<60)
					{
						pid_mode_switch(is_Free);
						turn=need2turn(getAngleZ(),nodesr.nextNode.angle);
						turnspeed=82*(180-fabs(need2turn(getAngleZ(),nodesr.nextNode.angle)))/360;
						while(fabs(turn)>4)
						{
							turn=need2turn(getAngleZ(),nodesr.nextNode.angle);
							turnspeed=84*(180-fabs(need2turn(getAngleZ(),nodesr.nextNode.angle)))/360;
							AdCircle(turnspeed,turn/1.4);
						}
						_flag=1;	
					}			
					else
					{
						num=motor_all.Distance;
						while(motor_all.Distance-num<15);
						angle.AngleT = nodesr.nextNode.angle;  //转绝对角度
						pid_mode_switch(is_Turn);	
						while(fabs(nodesr.nextNode.angle-getAngleZ())>2);						
						_flag=1;		
					}
				}
			}
			pid_mode_switch(is_Line);	
			nodesr.nowNode=nodesr.nextNode;	
			nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];//获取下一结点	
			
			//路程记录清零
			motor_all.Distance = 0;
			motor_all.encoder_avg = 0;
		} 
		else
		{		//如果路线走完
			
			motor_all.Cspeed=0;
			CarBrake();
			_flag=1;
			map.routetime+=1;
		}	
		
	}
		
	if(nodesr.flag&0x20)	//如果打到门是关的情况
	{				
		_flag=1;
		nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];		//对下一结点进行更新
		nodesr.nextNode.function = DOOR;
		nodesr.flag&=~0x20;
	}
}
	

void map_function(u8 fun)
{
	switch(fun)
	{
		case 0:break;
		case 1:break;		//寻线
		case UpStage	  :Stage();break;//平台
		case BBridge  	:Barrier_Bridge(150,30);break;//长桥 长度，速度
		case BHill	  	:Barrier_Hill(1);break;//楼梯
		case LBHill     :Barrier_Hill(2); break;  //双楼梯
		case SM         :Sword_Mountain();break;//刀山
		case View		    :view();break;//景点 后转
		case View1      :view1();break;//景点 直退
		case BACK       :back();break;
		case BSoutPole	:South_Pole(205);break;//南极
		case QQB		    :QQB_1();break;//跷跷板
		case BLBS       :Barrier_WavedPlate(50);break;//短减速板 速度，长度
		case BLBL		    :Barrier_WavedPlate(110);break;//长减速板 速度，长度
		case DOOR	  	  :door();break;//打门
		case BHM        :Barrier_HighMountain(); break;//上珠峰
		case Scurve	  	:S_curve();  break;
		case IGNORE     :ignore_node(); break;  //忽略该节点
		case UNDER      :undermou();break;
		default:break;		
	}
}


