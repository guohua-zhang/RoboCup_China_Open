#include "sin_generate.h"
#include "math.h"

#define PI 3.1415926

float sin_generator(struct sin_param *param)
{
	float output;
	
	param->actual_t = param->time * param->angular_velocity;
	
	output = param->gain * sin(param->actual_t * PI/180);
	
	++param->time;
	
	if (param->actual_t >= 360)
		param->time = 0;
	
	return output;
}




