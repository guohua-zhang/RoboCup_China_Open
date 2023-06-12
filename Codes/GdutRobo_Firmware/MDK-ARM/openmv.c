#include "openmv.h"
#include "main.h"
#include "stdio.h"
#include "bsp_motor.h"
#include "bsp_buzzer.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

static u8 openmv[SIZE] = {0};
static u8 mv_state = 0;
int value = 0;
uint8_t rx_len = 0;
u8 counter = 0;
u8 flag;

void openmv_receive_init(void)
{
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart2,openmv,4);
}

void USART2_IRQHandler(void)
{
	uint32_t flag_idle = 0;
	
	flag_idle = __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE); 
	if((flag_idle != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);

		HAL_UART_DMAStop(&huart2); 
		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);   
		rx_len = SIZE - temp; 
		
		if(mv_state==0 && openmv[0]==0x2C)
		{
			mv_state = 1;
		}
		else if(mv_state==1 && openmv[1]==0x12)
		{
			mv_state = 2;
		}
		else if(mv_state==2)
		{
			if(counter>=SIZE||openmv[3] == 0x5B)
			{
				mv_state = 3;
				flag = openmv[2];
			}
		}
		else if(mv_state == 3)
		{
			if(openmv[3] == 0x5B)
			{
				if(flag == 0x05)
				{
					value = 1;
				}
				else
				{
					value = 2;
				}
		
				mv_state = 0;
				counter = 0;
				for(int i = 0; i < SIZE; i++)
				{
					openmv[i] = 0;
				}
			}
			else
			{
				mv_state = 0;
				counter = 0;
				for(int i = 0; i < SIZE; i++)
				{
					openmv[i] = 0;
				}
			}
		}
	
	}
	HAL_UART_Receive_DMA(&huart2,openmv,4);
	HAL_UART_IRQHandler(&huart2);
}



