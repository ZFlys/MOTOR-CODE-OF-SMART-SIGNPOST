#include "led.h"


//初始化PB5和PE5为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOB, ENABLE);	 //使能PB,PC端口时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;				 //LEDR-->PB.0 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.0
							 	 //PB.0 输出高
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
