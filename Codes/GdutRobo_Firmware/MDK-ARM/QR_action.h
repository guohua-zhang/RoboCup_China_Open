#ifndef __QR_ACTION_H
#define __QR_ACTION_H

#include "QR_action.h"
#include "bsp_QRscan.h"
#include "bsp_servo_iic.h"
#include "turn.h"

void actions(uint8_t action);
void PreAction(void);
void Arrive (void);
void Treasure(void);
void Stand(void);
void Down_Body(void);
void myAction(void);
void usmart_set_servo(uint8_t choose,uint8_t angle);


#endif
