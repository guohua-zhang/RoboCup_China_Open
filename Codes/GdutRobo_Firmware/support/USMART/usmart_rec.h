#ifndef __USMART_REC_H
#define __USMART_REC_H

#include "sys.h"

#define USART_REC_LEN  			200  		//�����������ֽ��� 200
#define EN_USART1_RX 			1			//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; 	//���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         			//����״̬���	

#define RXBUFFERSIZE   1 					//�����С
extern u8 aRxBuffer[RXBUFFERSIZE];			//HAL��USART����Buffer

#endif

