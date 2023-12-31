#include "map_message.h"

//地图信息{结点名字，标志位，角度，长度（厘米），速度，功能函数}
//最后的参数，1为普通路线，其他均头文件有注释
//如/*S1	 1*/	{N3,FLEFT|M2L,160,280,100,1},	代表从，“寻右线，线数目由多变小认为到达路口”  “S1点指向N3点的角度为160度”，“长度为280cm，推荐速度为100”，“普通路口”
NODE Node[118]={
/*S1	 1*/	{N3,CLEFT,160,250,110,1},

/*P1 	 2*/	{N1,CRIGHT,180,40,60,1},	// step 25

/*N1 	 3*/	{P1,NO,0,30,70,UpStage},{B2,NO,135,40,40,BHill},{B1,NO,180,25,70,BBridge},

/*B1	 K*/	{N1,CLEFT,0,40,70,1},/* step 25 *//*{N1,NO,0,25,70,IGNORE},楼梯被撬掉*/{N2,CRIGHT|LEFT_LINE,180,70,90,1},

/*B2	 K*/	{N1},{N4,CLEFT,140,35,50,1},

/*B3	 K*/	{N2,CLEFT,-160,20,70,1},{N4,CLEFT,30,64,70,1},

/*N2 	 4*/	{B3,NO,30,30,70,BLBS}, {P2,NO,-178,86,70,UpStage},{B1,NO,0,70,90,BBridge},//速度原来是50  70
	
/*P2 	 5*/	{N2,/*CLEFT|DLEFT*/CLEFT|RIGHT_LINE,0,20,80,1},	// step 10		

/*S2   6*/	{N6,CLEFT,30,60,100,1},			

/*P3	 7*/	{N3,DRIGHT|CRIGHT,180,230,100,1},				//原来是130

/*N3 	 8*/	{S1,NO,-28,280,80,View}, {P3,NO,6,320,110,UpStage},/*step 300*/ {N10,DRIGHT,90,5,30,DOOR}, {N8,DRIGHT,140,25,70,DOOR}, {N4,CLEFT,180,120,100,1},	
	
/*N4 	 9*/	{B2}, {N3,DLEFT|CLEFT,0,120,100,1},/*step100*/ {N5,DRIGHT|CRIGHT,180,120,100,1}, /* 175 116 */{B3,NO,-150,64,70,BLBS},			
	
/*N5 	10*/	{N4,CRIGHT,0,120,110,1}, {N8,DLEFT|STOPTURN,45,160,70,1}, {N12,AWHITE|STOPTURN|CRIGHT,95,20,70,DOOR},{N6,CLEFT|CRIGHT,180,80,110,1},//原来是130	
	
/*N6 	11*/	{N5,DLEFT|CLEFT,0,80,110,1}, {C1,CRIGHT|DRIGHT|STOPTURN,150,200,90,1},{P4,NO,180,140,120,UpStage}, /*step 100*/{S2,NO,-140,80,85,View},
	
/*P4 	12*/	{N6,MUL2SING,0,155,100,1},

/*N7 	13*/	{P5,NO,90,50,60,UpStage}, {B9},{B8,LEFT_LINE,10,40,30,QQB},	

/*P5 	14*/	{N7,CLEFT,-90,20,50,1},	

/*B8    K*/     {N7},{N9,CRIGHT,150,30,30,1},

/*B9    K*/     {N7,MCLEFT,-45,20,40,1},{N9},

/*N8 	15*/	{N3,CRIGHT|CLEFT|STOPTURN,-45,10,70,1}, {N10,CRIGHT|CLEFT,38,186,70,1}, {N12,CRIGHT|CLEFT,150,190,80,1}, {N5,CRIGHT|CLEFT,-140,186,70,1},	
	
/*C1 	16*/	{N6,CRIGHT,-50,151,70,1}, {C2,NO,10,10,110,Scurve},	

/*C2 	17*/	{C1,CLEFT,170,154,70,1}, {N13,CRIGHT|CLEFT,120,40,90,1},//原来长度120标志位是MCLEFT

/*C3 	18*/	{N14,DLEFT,90,53,70,1}, {N9,MUL2SING,179,90,60,1},			

/*N9 	19*/	{C3,CLEFT,0,94,70,1}, {N10,RESTMPUZ|CLEFT|CRIGHT,180,194,70,1}, {B8},{B9,LEFT_LINE,-170,90,60,QQB},	
	
/*N10	20*/	{N9,CRIGHT,0,194,70,1}, {N15,DRIGHT|CRIGHT,90,15,80,1}, {N12,DLEFT|DRIGHT|RESTMPUZ,-180,264,70,1}, {N8,CLEFT|CRIGHT|DLEFT|DRIGHT,-140,136,70,1}, {N3,RESTMPUZ|DRIGHT|DRIGHT,-90,240,50,1}, 
	
/*N12	22*/	{N10,DLEFT|DRIGHT|CRIGHT|CLEFT,0,294,70,1}, {N16,DRIGHT,90,30,60,1}, {N13,CLEFT|CRIGHT,180,100,110,1}, {N5,CRIGHT|DRIGHT,-90,232,70,1}, {N8,CRIGHT|DLEFT,-43,186,70,1},		
	
/*N13   23*/	{N12,AWHITE,0,184,80,1}, {N18,CRIGHT|DRIGHT,50,110,90,1}, {P6,NO,180,90,70,UpStage}, {C2,CRIGHT,-45,15,70,Scurve},	
	
/*P6 	24*/	{N13,CLEFT|DRIGHT,0,110,85,1},		//70		

/*N14	25*/	{C3,DRIGHT|CRIGHT,-90,5,70,1}, {C7,DLEFT}, {S3,NO,-178,29,40,View1},				

/*S3 	26*/	{N14,NO,180,0,50,BACK},//{N14,DRIGHT|CRIGHT,0,20,50,1},	//{N14,NO,180,0,50,BACK},		

/*S4	27*/	{N15,NO,0,0,50,BACK},//{N15,DRIGHT|CRIGHT,180,20,50,1},	//{N15,NO,0,0,50,BACK},		

/*N15 28*/	{S4,NO,0,64,30,View1}, {C5,DLEFT,90,1,65,1}, {N10,DLEFT|DRIGHT,-90,5,60,1},		
	
/*S5 	29*/	{N16,NO,0,0,50,BACK},//{N16,DRIGHT|CRIGHT,180,20,50,1}, //{N16,NO,0,0,50,BACK},		

/*C4 	30*/	{C8,RESTMPUZ|DRIGHT|CRIGHT,90,90,90,UNDER}, /*长度原来160*/{N20,CRIGHT,150,204,70,1},			

/*C5 	31*/	{N15,DLEFT,-90,46,70,1},{B4,NO,180,40,100,IGNORE},

/*B4   K*/  {C5,DRIGHT,0,86,70,1},{N18,DLEFT|CLEFT|RESTMPUZ,180,210,100,1},

/*B5   K*/  {N18,DRIGHT,0,54,70,1},{N19,DRIGHT,180,144,70,1},

/*B6   K*/  {N20,CRIGHT|DRIGHT|RESTMPUZ,0,60,70,1},/*原标志位是CRIGHT*/{N22,DRIGHT,180,104,70,1}, 

/*B7   K*/  {N22,DLEFT|STOPTURN,0,70,80,1},/*原来长度是40速度60*/{C6},

/*N16 32*/  {S5,NO,0,52,40,View1}, {N12,DLEFT|DRIGHT,-90,10,70,1}, {N18,AWHITE,90,57,70,1},
	
/*N18	34*/	{B4,NO,0,86,70,IGNORE}, {B5,NO,180,54,70,BHill}, {N16,DLEFT|CLEFT,-90,10,90,1},	
	
/*N19	35*/	{B5,NO,0,144,70,BHill}, {C6,RESTMPUZ|DRIGHT|STOPTURN,90,75,80,1},/*长度79速度80*/ {N13,CRIGHT},	

/*P7 	36*/	{N20,RESTMPUZ|CLEFT,180,10,30,1},		//长度60							

/*N20   37*/	{C4,CLEFT,-45,210,70,1},/*长度原来204*/ {P7,NO,0,40,60,BHM}, {B6,NO,180,90,70,LBHill},
	
/*N22	39*/	{C9,DLEFT|CLEFT|STOPTURN,89,37,40,1},/*长度37*/ {B7,NO},{B6,RESTMPUZ,0,15,20,SM},	

/*C6 	40*/	{B7,NO,0,75,90,BHill}/*step 120 速度90 */, {N19,DLEFT},						

/*C7	41*/	{N14,DRIGHT|CRIGHT,-90,50,40,UNDER}, {C8,DLEFT,180,132,70,1},				

/*C8 	42*/	{C7,DRIGHT|CRIGHT,0,115,90,1}, {C4,CRIGHT,-90,189,80,1},				

/*C9	43*/	{N22,DLEFT|DRIGHT|AWHITE|STOPTURN,-90,20,50,1}, {P8,NO,180,20,45,BSoutPole},	//60速度	

/*P8 	44*/	{C9,DRIGHT|STOPTURN,0,5,50,1}													
};

//每个节点相邻结点的数目
unsigned char ConnectionNum[50] = {	//each node's connected node
	1, 1, 3, 2, 2, 2, 3, 1, 1, 1, 5, 4, 4, 4, 1, 3, 1, 2, 2, 4, 2, 2, 2, 4, 5, 5, 4, 1, 3, 1, 1, 3, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 1, 3, 3, 2, 2, 2, 2, 1
};

//每个节点的相邻结点的首地址
unsigned char Address[50] = {	//each type connected node's address
	0, 1, 2, 5, 7, 9, 11, 14, 15, 16, 17, 22, 26, 30, 34, 35, 38, 39, 41, 43, 47, 49, 51, 53, 57, 62, 67, 71, 72, 75, 76, 77, 80, 81, 83, 85, 87, 89, 91, 93, 96, 99, 102, 103, 106, 109, 111, 113, 115, 117
};


