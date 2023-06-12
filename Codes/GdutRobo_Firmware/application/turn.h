#ifndef __TURN_H
#define __TURN_H

#include "sys.h"
#include "bsp_imu.h"

struct Angle {
	float AngleT;
	float AngleG;
};

extern volatile struct Angle angle;

float need2turn(float nowangle,float targetangle);
void mpuZreset(float sensorangle ,float referangle);
float getAngleZ(void);
	
uint8_t Turn_Angle(float Angle);
void Turn_Angle_Relative(float Angle1);
uint8_t runWithAngle(float angle_want,float speed);
void AdCircle(float speed, float radius);

static inline float get_pitch(void)
{
	return imu.pitch;
}

void Turn_Angle360(void);

#endif

