#include "bsp_imu.h"
#include "main.h"
#include "stdio.h"

#define __USE_ATK_IMU

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
  
uint8_t imu_rx_buf[BUFFER_SIZE] = {0};
uint8_t imu_rx_len = 0;

volatile struct Imu imu;

void imu_receive_init(void)
{
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart3,imu_rx_buf,BUFFER_SIZE);
}

void USART3_IRQHandler(void)
{
	uint32_t flag_idle = 0;
	
	flag_idle = __HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE); 
	if((flag_idle != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);

		HAL_UART_DMAStop(&huart3); 
		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);   
		imu_rx_len = BUFFER_SIZE - temp; 
		
		if(imu_rx_buf[0] == 0x55)
		{
			uint8_t sum = 0;
			
			#ifdef __USE_JY61
			
			#if 0  //不需要读取加速度
			for(int i=0;i<10;i++) 
				sum += imu_rx_buf[i];
			if(sum == imu_rx_buf[10])
			{
				if(imu_rx_buf[1] == 0x51)
				{
					imu.ax = 16*(short)((imu_rx_buf[3]<<8)|imu_rx_buf[2])/32768.0;
					imu.ay = 16*(short)((imu_rx_buf[5]<<8)|imu_rx_buf[4])/32768.0;
					imu.az = 16*(short)((imu_rx_buf[7]<<8)|imu_rx_buf[6])/32768.0;
				}
				
			}
			sum = 0;
			#endif
			
			for(int i=22;i<32;i++)
				sum+=imu_rx_buf[i];
			if(sum == imu_rx_buf[32])
			{
				if(imu_rx_buf[23] == 0x53)
				{
					imu.pitch = 180.0*(short)((imu_rx_buf[25]<<8)|imu_rx_buf[24])/32768.0;
					imu.roll = 180.0*(short)((imu_rx_buf[27]<<8)|imu_rx_buf[26])/32768.0;
					imu.yaw = 180.0*(short)((imu_rx_buf[29]<<8)|imu_rx_buf[28])/32768.0;
					if(imu.yaw>360) imu.yaw -= 360;
					else if(imu.yaw<0) imu.yaw += 360;
					//printf("imu :%f  %f  %f\r\n",imu.roll,imu.pitch,imu.yaw);
				}
			}
			#endif
			
			#ifdef __USE_JY901
			for (int i=0; i<10; i++)
				sum += imu_rx_buf[i];
			if (sum == imu_rx_buf[10])
			{
				if (imu_rx_buf[1] == 0x53)
				{
					//根据陀螺仪的安装方向确定roll和pitch
					imu.pitch = 180.0*(short)((imu_rx_buf[3]<<8)|imu_rx_buf[2])/32768.0;  
					imu.roll = 180.0*(short)((imu_rx_buf[5]<<8)|imu_rx_buf[4])/32768.0;
					imu.yaw = 180.0*(short)((imu_rx_buf[7]<<8)|imu_rx_buf[6])/32768.0;
					if(imu.yaw>360) imu.yaw -= 360;
					else if(imu.yaw<0) imu.yaw += 360;
					//printf("%d,%d,%d\r\n", (int)imu.roll, (int)imu.pitch, (int)imu.yaw);
				}
			}
			#endif
			
			#ifdef __USE_ATK_IMU
			for (int i=0; i<10; i++)
				sum += imu_rx_buf[i];
			if (sum == imu_rx_buf[10])
			{
				if (imu_rx_buf[2] == 0X01)
				{
					//根据陀螺仪的安装方向确定roll和pitch
					imu.roll = 180.0*(short)((imu_rx_buf[5]<<8)|imu_rx_buf[4])/32768.0;    //原来是pitch
					imu.pitch = 180.0*(short)((imu_rx_buf[7]<<8)|imu_rx_buf[6])/32768.0;   //原来是roll
					imu.yaw = 180.0*(short)((imu_rx_buf[9]<<8)|imu_rx_buf[8])/32768.0;
					if(imu.yaw>360) imu.yaw -= 360;
					else if(imu.yaw<0) imu.yaw += 360;
					//imu.yaw = -imu.yaw;   //航向角取反
					imu.pitch = -imu.pitch; //仰角取反
					
//					printf("%d,%d,%d\r\n", (int)imu.roll, (int)imu.pitch, (int)imu.yaw);
				}
			}
			#endif
			
		}
		imu_rx_len = 0;
		memset(imu_rx_buf,0,imu_rx_len);
	}
	HAL_UART_Receive_DMA(&huart3,imu_rx_buf,BUFFER_SIZE);
	HAL_UART_IRQHandler(&huart3);
}



