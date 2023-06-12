#include "bsp_servo_iic.h"
#include "main.h"
#include "math.h"

//GPIO simulate IIC
#define READ_SDA() HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)
#define SDA_HIGH() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET)
#define SDA_LOW()  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET)
#define SCL_HIGH() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET)
#define SCL_LOW()  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET)
#define SDA_IN()  {GPIOB->CRL&=0XFFFFFFF0;GPIOB->CRL|=(uint32_t)8;} 
#define SDA_OUT() {GPIOB->CRL&=0XFFFFFFF0;GPIOB->CRL|=(uint32_t)3;}

//IIC operating function			 
void iic_start(void);				
void iic_stop(void);	  			
void iic_send_byte(uint8_t txd);			
void iic_ack(void);					
void iic_not_ack(void);
uint8_t iic_wait_ack(void); 
uint8_t iic_read_byte(unsigned char ack);

//pca9685
void pca_write(uint8_t adrr,uint8_t data);
uint8_t pca_read(uint8_t adrr);
void pca_setfreq(float freq);
void pca_setpwm(uint8_t num, uint32_t on, uint32_t off);

void iic_start(void)
{
	SDA_OUT();     
	SDA_HIGH();	
	SCL_HIGH();	
	delay_us(4);
	SDA_LOW();
	delay_us(4);
	SCL_LOW();
}	  

void iic_stop(void)
{
	SDA_OUT();
	SCL_LOW();
	SDA_LOW();
 	delay_us(4);
	SCL_HIGH(); 
  SDA_HIGH();
	delay_us(4);							   	
}

uint8_t iic_wait_ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      
	SDA_HIGH();delay_us(1);	   
	SCL_HIGH();delay_us(1);	 
	while(READ_SDA())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			iic_stop();
			return 1;
		}
	}
	SCL_LOW();
	return 0;  
} 

void iic_ack(void)
{
	SCL_LOW();
	SDA_OUT();
	SDA_LOW();
	delay_us(2);
	SCL_HIGH();
	delay_us(2);
	SCL_LOW();
}
	    
void iic_not_ack(void)
{
	SCL_LOW();
	SDA_OUT();
	SDA_HIGH();
	delay_us(2);
	SCL_HIGH();
	delay_us(2);
	SCL_LOW();
}					 				     
		  
void iic_send_byte(uint8_t txd)
{                        
    uint8_t t;   
	  GPIO_PinState pin_state;
		SDA_OUT(); 	    
    SCL_LOW();
    for(t=0;t<8;t++)
    {              
			pin_state = ((txd&0x80)>>7 == 0)? GPIO_PIN_RESET : GPIO_PIN_SET;
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,pin_state);
					txd<<=1; 	  
			delay_us(2);   
			SCL_HIGH();
			delay_us(2); 
			SCL_LOW();	
			delay_us(2);
    }	 
} 	    

uint8_t iic_read_byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();
    for(i=0;i<8;i++ )
	{
        SCL_LOW(); 
        delay_us(2);
				SCL_HIGH();
        receive<<=1;
        if(READ_SDA())receive++;   
				delay_us(1); 
    }					 
    if (!ack)
        iic_not_ack();
    else
        iic_ack(); 
    return receive;
}

void pca_write(uint8_t adrr,uint8_t data)
{ 
	iic_start();
	
	iic_send_byte(pca_adrr);
	iic_wait_ack();
	
	iic_send_byte(adrr);
	iic_wait_ack();
	
	iic_send_byte(data);
	iic_wait_ack();
	
	iic_stop();
}

uint8_t pca_read(uint8_t adrr)
{
	uint8_t data;
	iic_start();
	
	iic_send_byte(pca_adrr);
	iic_wait_ack();
	
	iic_send_byte(adrr);
	iic_wait_ack();
	
	iic_start();
	
	iic_send_byte(pca_adrr|0x01);
	iic_wait_ack();
	
	data=iic_read_byte(0);
	iic_stop();
	
	return data;
}

void pca_setfreq(float freq)
{
		uint8_t prescale,oldmode,newmode;
		double prescaleval;
		freq *= 0.92; 
		prescaleval = 25000000;
		prescaleval /= 4096;
		prescaleval /= freq;
		prescaleval -= 1;
		prescale =floor(prescaleval + 0.5f);

		oldmode = pca_read(pca_mode1);
	
		newmode = (oldmode&0x7F) | 0x10; // sleep
	
		pca_write(pca_mode1, newmode); // go to sleep
	
		pca_write(pca_pre, prescale); // set the prescaler
	
		pca_write(pca_mode1, oldmode);
		HAL_Delay(2);
	
		pca_write(pca_mode1, oldmode | 0xa1); 
}

void pca_setpwm(uint8_t num, uint32_t on, uint32_t off)
{
		pca_write(LED0_ON_L+4*num,on);
		pca_write(LED0_ON_H+4*num,on>>8);
		pca_write(LED0_OFF_L+4*num,off);
		pca_write(LED0_OFF_H+4*num,off>>8);
}

/**
  * @brief          init pca9685
  * @param[in]      hz:frequence ; angle: Initialization Angle
  * @retval         none
  */
void pca_init(float hz,uint8_t angle)
{
	
	pca_write(pca_mode1,0x0);
	pca_setfreq(hz);
	delay_ms(500);
}

/**
  * @brief          Set the absolute angle of servo
  * @param[in]      num: servo num ; angle:Absolute Angle
  * @retval         none
  */
void angle_write(uint8_t num,uint8_t angle)
{
	uint32_t off = (uint32_t)(158+angle*2.2);
	pca_setpwm(num,1,off);
}


