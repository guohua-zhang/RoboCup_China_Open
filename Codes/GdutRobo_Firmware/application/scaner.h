#ifndef __SCANER_H
#define __SCANER_H
#include "sys.h"
#include "pid.h"

typedef struct scaner	
{
	uint32_t detail;
	float pow;
	u8 ledNum;
	u8 lineNum;
}SCANER;


struct Scaner_Set {
	float CatchsensorNum;
	int8_t EdgeIgnore;
};

extern struct Scaner_Set scaner_set;

extern SCANER Scaner;
extern const float line_weight[16];

void powCal(SCANER *scaner, unsigned char sensorNum, int8_t edge_ignore);

void Go_Line(float speed);

#endif
