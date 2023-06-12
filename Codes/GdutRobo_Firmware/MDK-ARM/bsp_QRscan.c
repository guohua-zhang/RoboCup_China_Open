#include "bsp_QRscan.h"
#include "main.h"
#include "usart.h"
#include "map_message.h" 
#include "QR_action.h"
#include "bsp_servo_iic.h"
#include "bsp_buzzer.h"
#include "stdio.h"


u16 BAOWU_flag=0;
u16 BAOWU_flag_2=0;
u16 BAOWU_flag_3=0;
int ct_flag = 0;
int BW_num[3] = {0,0,0};


void QR_receive_init(void)
{
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);     //开启接收完成中断
}

void UART5_IRQHandler(void)
{
	uint8_t res;
	
	//buzzer_on();
	
	if ((__HAL_UART_GET_FLAG(&huart5,UART_FLAG_RXNE) != RESET))
	{
		
		res = huart5.Instance->DR;
		ct_flag = 1;
		//printf("%d\r\n",res);  //检验一下QR的扫描值，注意要对应ASCII表
		
		switch (res)
		{
			case 0x33: 
				BAOWU_flag |= 0x04;
				BW_num[0] = BAOWU_flag;
				break;
			case 0x34: 
				BAOWU_flag |= 0x08;   
				BW_num[0] = BAOWU_flag;
				break;
			case 0x35: 
				BAOWU_flag_2 |= 0x10; 
				BW_num[1] = BAOWU_flag_2;
				break; 
			case 0x36: 
				BAOWU_flag_2 |= 0x20; 
				BW_num[1] = BAOWU_flag_2;
				break;
			case 0x37: 
				BAOWU_flag_3 |= 0x40; 
				BW_num[2] = BAOWU_flag_3;
				break;
			case 0x38: 
				BAOWU_flag_3 |= 0x80; 
				BW_num[2] = BAOWU_flag_3;
				break;
		
		}
		
		
		
	}
	
			
}


int check_BW(u16 node)
{
	switch(node)
	{
		case P3:
			if(BW_num[0]&0x04) return 1;else return 0;//3   
		case P4:
			if(BW_num[0]&0x08) return 1;else return 0;//4
		case P5:
			if(BW_num[1]&0x20) return 1;else return 0;//5
		case P6:
			if(BW_num[1]&0x10) return 1;else return 0;//6
		case P7:
			if(BW_num[2]&0x80) return 1;else return 0;//7
		case P8:
			if(BW_num[2]&0x40) return 1;else return 0;//8
		default: return 0;
	}
		
}


