#include "bsp_led.h"
#include "main.h"

//������ʾ��
void LED_on(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
}

void LED_off(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
}

void Shine_on(void)  //׼����������˸
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
}

void Shine_off(void)  //׼����������˸
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
}

void LED_twinkle(void)
{
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
}
//׼��������˸һ��
void Shine_o_f(void)
{
	HAL_Delay(100);
	Shine_on();
	HAL_Delay(200);
	Shine_off();
}

void usmart_led(uint8_t sw)
{
	if (sw)
		LED_on();
	else
		LED_off();
}

