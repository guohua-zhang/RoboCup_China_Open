/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "bsp_imu.h"
#include "openmv.h"
#include "bsp_buzzer.h"
#include "bsp_led.h"
#include "bsp_servo_iic.h"
#include "bsp_delay.h"
#include "bsp_motor.h"
#include "bsp_linefollower.h"
#include "pid.h"
#include "usmart.h"
#include "sin_generate.h"
#include "turn.h"
#include "tim_it.h"
#include "map.h"
#include "barrier.h"
#include "speed_ctrl.h"
#include "motion.h"
#include "QR_action.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float mpuZ_reset_val = 0;
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
	
	delay_init();
	pca_init(50,90);
	motor_pwm_enable();
	LFB_receive_init();
	pid_init();
	imu_receive_init();
	openmv_receive_init();
	QR_receive_init();
	
	mapInit();
	delay_ms(500);
	delay_ms(500);
	//陀螺仪角度复位，采样10次取平均值
	for (int i = 0; i<10; i++)
	{
		delay_ms(20);
		mpuZ_reset_val += imu.yaw;   
		imu.compensatePitch += imu.pitch;
	}
	mpuZ_reset_val /= 10;   // 这是当前的0°角
	mpuZreset(mpuZ_reset_val, nodesr.nowNode.angle);
	imu.compensatePitch /= 10;
	
	HAL_TIM_Base_Start_IT(&htim6);  //使能定时器6中断
	
	LED_on();          //亮灯
	
	
	/*******机器人的起始动作********/
	Shine_o_f();       //闪灯一次
	actions(5);        //起身
  HAL_Delay(500);
	actions(0);        //两次摇头
//	actions(6);        //落下
	buzzer_on();
	HAL_Delay(200);
	buzzer_off();
	/*******************************/
	
//	myAction();
	
//					/******************电机检测程序*********************/		
//					while(1)
//					{
//							motor_set_pwm(1,-2000);
//							motor_set_pwm(2,-2000);
//							motor_set_pwm(3,-2000);
//							motor_set_pwm(4,-2000);
//					}
//				/****************************************************/	
	
	//等待禁行板
	while (Infrared_ahead == 0);  //此时前方无挡板
	while (Infrared_ahead == 1);   //此时前方有挡板
	
	//if(nodesr.nowNode.nodenum==N3)//为测试方便，如果是P2平台，才需要做准备动作
		
		
		pid_mode_switch(is_Line);
		motor_all.Cspeed = 40;
		
		/******判断小车正在下平台*******/
//	  while (imu.pitch < 10);
//		while (imu.pitch > 10);
		actions(6);        //落下
		
		/*******************************/
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Cross();
//	  if (route[map.point-1]==0XFF && map.routetime==1)
//	  {

//			/*******机器人的起始动作********/
//			Shine_o_f();       //闪灯一次
//			actions(5);        //起身
//			actions(0);        //两次摇头
//			actions(6);        //落下
//			buzzer_on();
//			HAL_Delay(200);
//			buzzer_off();
//			/*******************************/
//			
//			nodesr.nowNode.nodenum = N2;		//起始点
//			nodesr.nowNode.angle = 0;		//起始角度
//			nodesr.nowNode.function = 1;	//起始函数
//			nodesr.nowNode.speed = 80;//60
//			nodesr.nowNode.step=40;//30
//			nodesr.nowNode.flag = CLEFT|RIGHT_LINE;
//		  
//		  //等待禁行板
//		  while (Infrared_ahead == 0);
//		  while (Infrared_ahead == 1);
//		  ++map.routetime;
//		  map.point = 0;
//		  
//		  motor_all.Cspeed = 10;
//		  pid_mode_switch(is_Line);
//		  motor_all.Cincrement = 1.5;
//		  motor_all.Gincrement = 2;
//		  while (imu.pitch < 10);
//		  while (imu.pitch > 10);
//	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	buzzer_on();
	delay_ms(2000);
	buzzer_off();

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
