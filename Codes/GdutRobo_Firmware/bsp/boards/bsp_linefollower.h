#ifndef __BSP_LINEFOLLOWER_H
#define __BSP_LINEFOLLOWER_H

#include "sys.h"

#define LFB_SENSOR_NUM 16   //循迹板传感器数量
#define LFB_HALF_SENSOR_NUM 8

//向前的红外
#define Infrared_ahead  (uint8_t)!PBin(2)
#define Infrared_FrontLeft  (uint8_t)!PAin(12)

struct Line_Grays
{
	uint32_t Line;  //循迹值
	float Value;  //差值
	uint8_t LED_num;  //亮灯数
	int Line_width;  //线数
	uint8_t Line_num;  //线数
	float Line_Middle;  //循线中心
};

struct Infrared_Sensor    //红外传感器
{
	uint8_t outside_left;
	uint8_t outside_right;
	uint8_t inside_left;
	uint8_t inside_right;
};

extern struct Line_Grays LFB_data;
extern volatile struct Infrared_Sensor infrared;

//串口接收循迹板初始化
void LFB_receive_init(void);

//向地址为1的循迹板发送查询指令
void LFB_send(void);

#endif

