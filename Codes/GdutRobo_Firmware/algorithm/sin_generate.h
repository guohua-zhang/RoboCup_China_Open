#ifndef __SIN_GENERATE_H
#define __SIN_GENERATE_H

#include "sys.h"

struct sin_param {
	uint16_t time;
	float actual_t;
	float gain;
	float angular_velocity;
};

float sin_generator(struct sin_param *param);

#endif

