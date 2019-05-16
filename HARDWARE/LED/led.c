#include "led.h"


//��ʼ��PB5��PE5Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOB, ENABLE);	 //ʹ��PB,PC�˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;				 //LEDR-->PB.0 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.0
							 	 //PB.0 �����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_4);
	GPIO_SetBits(GPIOC, GPIO_Pin_5);
	GPIO_SetBits(GPIOB, GPIO_Pin_0);

}

void red_led_ctl(int state)
{
	if(state == 1)
		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	else if(state == 0)
		GPIO_SetBits(GPIOB, GPIO_Pin_0);
}

void green_led_ctl(int state)
{
	if(state == 1)
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
	else if(state == 0)
		GPIO_SetBits(GPIOC, GPIO_Pin_4);
}

void blue_led_ctl(int state)
{
	if(state == 1)
		GPIO_ResetBits(GPIOC, GPIO_Pin_5);
	else if(state == 0)
		GPIO_SetBits(GPIOC, GPIO_Pin_5);
}

void yellow_led_ctl(int state)
{
	if(state == 1) {
		red_led_ctl(1);
		green_led_ctl(1);
	} else if(state == 0) {
		red_led_ctl(0);
		green_led_ctl(0);
	}
}

void magenta_led_ctl(int state)
{
	if(state == 1) {
		red_led_ctl(1);
		blue_led_ctl(1);
	} else if(state == 0) {
		red_led_ctl(0);
		blue_led_ctl(0);
	}
}

void cyan_led_ctl(int state)
{
	if(state == 1) {
		green_led_ctl(1);
		blue_led_ctl(1);
	} else if(state == 0) {
		green_led_ctl(0);
		blue_led_ctl(0);
	}
}
