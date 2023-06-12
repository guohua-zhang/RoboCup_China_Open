#ifndef __PID_H
#define __PID_H

#include "sys.h"

struct I_pid_obj {
	float output;
	int bias;
	int last_bias;
	int last2_bias;
	int measure;
	int target;
};

struct P_pid_obj {
	float output;
	float bias;
	float measure;
	float last_bias;
	float integral;
	float last_differential;
	float target;
};

//differential_filterK: 微分项滤波系数，取值范围(0,1]
//系数越小滤波效应越大，当系数为1时不进行滤波
struct PID_param {
	float kp;
	float ki;
	float kd;
	float differential_filterK;
	float outputMin;
	float outputMax;
	float actualMax;
};

extern struct I_pid_obj motor_L0;
extern struct I_pid_obj motor_L1;
extern struct I_pid_obj motor_R0;
extern struct I_pid_obj motor_R1;
extern struct PID_param motor_pid_param;

extern struct P_pid_obj line_pid_obj;
extern struct PID_param line_pid_param;

extern struct P_pid_obj gyroT_pid, gyroG_pid;
extern struct PID_param gyroT_pid_param, gyroG_pid_param;

void incremental_PID (struct I_pid_obj *motor, struct PID_param *pid);
float positional_PID (struct P_pid_obj *obj, struct PID_param *pid);
void pid_init(void);
void motor_pid_clear(void);

void usmart_pid(uint16_t val,int deno,int mode);

#endif
