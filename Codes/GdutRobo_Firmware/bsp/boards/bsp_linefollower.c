#include "bsp_linefollower.h"
#include "usart.h"
#include "tim.h"
#include "scaner.h"
#include "stdio.h"

#define FRAME_HEAD 0XAC
#define RECEIVE_LEN 5

volatile struct Infrared_Sensor infrared;

void show_line(SCANER *gray);


void LFB_receive_init(void)
{
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);     //开启接收完成中断
}


//向循迹板发送查询指令
void LFB_send(void)
{
	uint8_t getlinecom[2]={FRAME_HEAD, 0X2B};
	HAL_UART_Transmit(&huart4, getlinecom, 2, 0XFF);
}


void UART4_IRQHandler(void)
{
	uint8_t Res;
	static uint8_t RxBuffer[20];
	static uint8_t data_cnt;
	static uint8_t state;
	register uint8_t sum = 0, i = 0;
	
	if ((__HAL_UART_GET_FLAG(&huart4,UART_FLAG_RXNE)!=RESET))
	{
		Res = huart4.Instance->DR;
		
		if(state == 1)
		{
			RxBuffer[data_cnt++] = Res;
			if (data_cnt >= RECEIVE_LEN)
			{
				for (i=0; i<RECEIVE_LEN-1; i++)
					sum += RxBuffer[i];
				
				if (sum == RxBuffer[RECEIVE_LEN-1])
				{
					__HAL_UART_DISABLE_IT(&huart4, UART_IT_RXNE);  //关闭接收中断
					
					Scaner.detail = (RxBuffer[2])|(RxBuffer[1]<<8);

					infrared.outside_left = RxBuffer[3]&0X01;
					infrared.outside_right = (RxBuffer[3]&0X02)>>1;
					infrared.inside_left = (RxBuffer[3]&0X04)>>2;
					infrared.inside_right = (RxBuffer[3]&0X08)>>3;
					
					powCal(&Scaner, LFB_SENSOR_NUM, scaner_set.EdgeIgnore);
					//show_line(&Scaner);    //检查循迹灯的返回值
					__HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);  //开启接收中断
				}
				state = 0;
			}
		}
		else if (state==0 && Res==FRAME_HEAD)
		{
			state=1;
			data_cnt = 0;
			RxBuffer[data_cnt++] = Res;
		}
		else
			state=0;
	}
}


void show_line(SCANER *gray)
{
	printf("%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d\r\n", 
		(gray->detail&0X01), (gray->detail&0X02)>>1, (gray->detail&0X04)>>2, (gray->detail&0X08)>>3, 
		(gray->detail&0X10)>>4, (gray->detail&0X20)>>5, (gray->detail&0X40)>>6, (gray->detail&0X80)>>7,
		(gray->detail&0X100)>>8, (gray->detail&0X200)>>9, (gray->detail&0X400)>>10, (gray->detail&0X800)>>11,
		(gray->detail&0X1000)>>12, (gray->detail&0X2000)>>13, (gray->detail&0X4000)>>14, (gray->detail&0X8000)>>15);
}








