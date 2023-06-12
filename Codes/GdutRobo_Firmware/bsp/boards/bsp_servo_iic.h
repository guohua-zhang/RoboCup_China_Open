#ifndef BSP_SERVO_IIC_H
#define BSP_SERVO_IIC_H
#include "sys.h"
#include "bsp_delay.h"
				
//pca9685
#define pca_adrr 0xAA

#define pca_mode1 0x0
#define pca_pre 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define jdMIN  115 
#define jdMAX  590 
#define jd000  130 //0 degrees ~ (4096)
#define jd180  520 //180 degrees ~ (4096)

extern void pca_init(float hz,uint8_t angle);
extern void angle_write(uint8_t num,uint8_t angle);//angle:Absolute Angle

#endif

