#include "QR_action.h"
#include "bsp_QRscan.h"
#include "bsp_servo_iic.h"
#include "turn.h"
#include "bsp_led.h"

//使用通道 0 -> 身   10   直立  100  放下
//使用通道 1 -> 左手  0   举高  160 放下
//使用通道 2 -> 右手  160 举高  10  放下
//使用通道 3 -> 头    0   160


void actions(uint8_t action)
{
	switch (action)
	{
		case 0://两次摇头
			//第一次摇头
			angle_write(3, 0);
			delay_ms(400);
			angle_write(3, 160);
			delay_ms(1000);
			
			//第二次摇头
			angle_write(3, 0);
			delay_ms(400);
			angle_write(3, 160);
			delay_ms(1000);
		
			break;
		
		case 1://举手
			angle_write(1, 0);
			angle_write(2, 160);
			break;
		
		case 2://放手
			angle_write(1, 160);
			angle_write(2, 0);
			break;
		
		case 3://先左手，后右手
			//举起
			angle_write(1, 0);
			delay_ms(200);//300
			angle_write(2, 160);
			delay_ms(200);
			//放下
			angle_write(1, 160);
			delay_ms(200);
		  angle_write(2, 0);
			break;
		
//		case 4://先右手，后左手
//			//举起
//			angle_write(2, 160);
//			delay_ms(300);
//			angle_write(1, 10);
//			delay_ms(300);
//			//放下
//			angle_write(2, 0);
//			delay_ms(300);
//		  angle_write(1, 160);
//			break;	
		
		case 5://直立
			angle_write(0,10);
			break;
		
		case 6://身体放下
			angle_write(0,100);
			break;
		
	}	
}

//保持直立
void Stand (void)
{
	actions(5);
	actions(2);
}

//放下身体
void Down_Body(void)
{
	actions(6);
	actions(2);
}

//机器人准备动作  两次摇头
void PreAction(void)
{
	actions(0);
}

void Arrive (void)//直立景点动作
{
	actions(5);
	actions(3);
}

void Treasure(void)//找到宝物动作
{
	actions(5);
	actions(1);
	//旋转360
	
	//actions(2);
}

void myAction(void)
{
	Shine_o_f();       //闪灯一次
	
	//景点动作
	actions(5);  
	delay_ms(100);//800
	actions(3);
	delay_ms(200); 
	Turn_Angle_Relative(179);
	if(check_BW(nodesr.nowNode.nodenum)) //检测宝物
		{
			actions(1);
			delay_ms(100);//500
			Turn_Angle360();
			actions(2);
//			delay_ms(500);
		}
	actions(6);
//	delay_ms(100);
}

void usmart_set_servo(uint8_t choose, uint8_t angle)
{
	angle_write(choose,angle);
}

