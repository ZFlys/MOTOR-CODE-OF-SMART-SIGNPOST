#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
#define RED    {	GPIO_SetBits(GPIOB, GPIO_Pin_0);	GPIO_SetBits(GPIOC, GPIO_Pin_5);	GPIO_ResetBits(GPIOC, GPIO_Pin_4);			}
#define GREEN  {	GPIO_ResetBits(GPIOB, GPIO_Pin_0);	GPIO_SetBits(GPIOC, GPIO_Pin_5);	GPIO_SetBits(GPIOC, GPIO_Pin_4);			}
#define BLUE   {	GPIO_SetBits(GPIOB, GPIO_Pin_0);	GPIO_ResetBits(GPIOC, GPIO_Pin_5);	GPIO_SetBits(GPIOC, GPIO_Pin_4);			}
void LED_Init(void);//��ʼ��
void red_led_ctl(int state);
void green_led_ctl(int state);
void blue_led_ctl(int state);
void yellow_led_ctl(int state);
void magenta_led_ctl(int state);
void cyan_led_ctl(int state);
		 				    
#endif
