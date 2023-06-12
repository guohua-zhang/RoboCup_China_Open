#ifndef __MAP_H
#define __MAP_H
#include "sys.h"

#include "map_message.h"

//#define FLEFT 	0x00	//左寻线

#define NO      	(1<<0) 
#define DLEFT 		(1<<1)	//左边横线
#define DRIGHT 		(1<<2)	//右边横线
#define CLEFT		  (1<<3)	//左边斜线
#define CRIGHT		(1<<4)	//右边斜线
#define MUL2SING	(1<<5)	//多条变一条
#define MUL2MUL 	(1<<6)   //多条变多条
#define AWHITE  	(1<<7)		//全黑

#define RESTPID	    (1<<8)
#define RESTMPUZ	(1<<9)		//陀螺仪校准
#define STOPTURN 	(1<<10)
#define SLOWDOWN	(1<<10)

#define LEFT_LINE      /* (1<<11)*/(1<<12)
#define RIGHT_LINE		/*(1<<12)*/(1<<13)
#define MCLEFT       (1<<13)
#define MCRIGHT      (1<<14)
//#define Thirdline		0x30

enum barriers {
	UpStage = 2,
	BBridge,
	BHill,
	LBHill,
	SM,
	View,
	View1,
	BACK,
	BSoutPole,
	QQB,
	BLBS,
	BLBL,
	DOOR,
	BHM,
	Scurve,
	IGNORE,
	UNDER,
};

extern u8 door1route[100];
extern u8 door2route[100];
extern u8 door3route[100];
extern u8 door4route[100];
extern u8 route[100];


enum MapNode {	//MapNode
	S1, P1, N1,B1,B2,B3,N2, P2, S2,
	P3, N3, N4, N5, N6, P4,
	N7,
	P5, B8, B9, N8, C1, C2,
	C3, N9, N10, N12, N13, P6,
	N14, S3, S4, N15, S5,
	C4, C5, B4, B5, B6, B7, 
	N16, N18, N19,
	P7, N20, N22, C6,
	C7, C8, C9, P8,
};

/**************************************/
//结点信息
//flag 0位寻线方式：0左寻线，1右寻线
//flag 123位到达路口标志：	000最左边打到，001最右边打到，010左边数线，011右边数线，100线数由多变成一条	
//flag 45位，数线数目	
//flag 6位，寻线方式是否要切换，1需要切换，0不需要切换
//flag 7位	需要陀螺仪校正
//flag 8~11	
typedef struct _node{
	u8 nodenum;     //结点名称
	u16  flag;	    //结点标志位
	float angle;	//角度	
	u16	step;		//线长
	float speed;	//寻线速度
	u8 function;    //结点函数
}NODE;

extern NODE Node[118];
/*************************/
//flag 0位：1编码器清零请求，0清零完毕
//flag 1位：启动路口判断
//flag 2位：是否到达路口
//flag 3位：arrive里temp清零
//flag 4位：Z轴置零
//flag 5位：路线处理复位 打到门
//flag 6位：没有门
typedef struct _nodesr{
	u8 flag;
	NODE nowNode;		//当前结点
	NODE nextNode;	//下一结点
}NODESR;

extern NODESR nodesr;

struct Map_State {
	u8 point;
	u8 routetime;
};
extern struct Map_State map;


u8 getNextConnectNode(u8 nownode,u8 nextnode);
void mapInit(void);
void Cross(void);
void map_function(u8 fun);
u8 deal_arrive(void);


#endif







