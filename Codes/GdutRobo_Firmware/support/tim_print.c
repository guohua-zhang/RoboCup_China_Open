#include "tim.h"
#include "tim_it.h"
#include "bsp_buzzer.h"
#include "turn.h"
#include "bsp_imu.h"
#include "speed_ctrl.h"
#include "my_print.h"

extern volatile uint8_t PIDMode;

void TIM8_UP_IRQHandler(void)
{
	if (TIM8->SR & 0X0001)
	{
		printf("%f\r\n", motor_all.Lspeed);
	}
	TIM8->SR &= ~(1<<0);
}





