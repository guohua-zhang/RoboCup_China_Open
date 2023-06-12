#include "bsp_buzzer.h"
#include "main.h" 

void buzzer_on(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
}

void buzzer_off(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
}
