#include "ican_test.h"
#include "bsp_servo_iic.h"

void ican_servo_up(uint8_t mode)
{
	switch (mode)
	{
		case 1: angle_write(11, 0); break;
		case 2: angle_write(10, 50); angle_write(9, 130); break;
		default: ;
	}
}

void ican_servo_down(uint8_t mode)
{
	switch (mode)
	{
		case 1: angle_write(11, 100); break;
		case 2: angle_write(10, 130); angle_write(9, 50); break;
		default: ;
	}
}

