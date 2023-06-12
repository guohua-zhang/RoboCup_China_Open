#ifndef __BSP_QRSCAN_H
#define __BSP_QRSCAN_H

#include "bsp_QRscan.h"
#include "main.h"
#include "usart.h"
#include "map_message.h" 

#define FRAME_HEAD 0XAC
#define RECEIVE_LEN 6



void UART5_IRQHandler(void);
void QR_receive_init(void);
int check_BW(u16 node);

#endif
