#include "key.h"
#include "motor.h"
#include "led.h"
#include "can.h"
volatile u8 Tst_Mode=0; //记录当前电机的测试状态
void Key_GpioConfig(){

			GPIO_InitTypeDef  GPIO_InitStructure;
			NVIC_InitTypeDef NVIC_InitStructure;
			EXTI_InitTypeDef exti;
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;				
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 		 
			//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
			//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);		
			SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
			exti.EXTI_Line=EXTI_Line0;
			exti.EXTI_Mode=EXTI_Mode_Interrupt;
			exti.EXTI_Trigger=EXTI_Trigger_Falling;
			exti.EXTI_LineCmd=ENABLE;
			EXTI_Init(&exti);
			NVIC_InitStructure.NVIC_IRQChannel =EXTI0_IRQn;  
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
			NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;  //从优先级3级
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
			NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
			
				SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource1);	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource1);
			exti.EXTI_Line=EXTI_Line1;
			exti.EXTI_Mode=EXTI_Mode_Interrupt;
			exti.EXTI_Trigger=EXTI_Trigger_Falling;
			exti.EXTI_LineCmd=ENABLE;
			EXTI_Init(&exti);
			NVIC_InitStructure.NVIC_IRQChannel =EXTI1_IRQn;  
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
			NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;  //从优先级3级
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
			NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
			
			SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource2);
			exti.EXTI_Line=EXTI_Line2;
			exti.EXTI_Mode=EXTI_Mode_Interrupt;
			exti.EXTI_Trigger=EXTI_Trigger_Falling;
			exti.EXTI_LineCmd=ENABLE;
			EXTI_Init(&exti);
			NVIC_InitStructure.NVIC_IRQChannel =EXTI2_IRQn;  
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
			NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;  //从优先级3级
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
			NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


}
/***********************************************************/
/***********************************************************/
/*********正转测试******************************************/
/***********************************************************/
void EXTI0_IRQHandler(){
	
	
//	Prameter.angle_current=180;
//	EXTI_ClearITPendingBit(EXTI_Line0);
//	return;
	
	if(Prameter.staus==1){
	
			delay_ms(500);
			EXTI_ClearITPendingBit(EXTI_Line0);
		  return;
	}
		
	if(Tst_Mode==0){
  
		
		MOTORRESET;
		MOTORENABLE;
		DIRPOSITIVE;
	  Motor_Start();
    Speed_Change(200);
		RED;
		BLUE;

	}
	else{
	   
			if(Tst_Mode!=1){
						
						PAUSE();
						MOTORRESET;
						MOTORENABLE;
						DIRPOSITIVE;
						Motor_Start();
						Speed_Change(200);
						RED;
						BLUE;
			}
	
	}
	Tst_Mode=1;
	delay_ms(500);
	EXTI_ClearITPendingBit(EXTI_Line0);
}
/*******************************************************/
/*******************************************************/
/************反转测试***********************************/
void EXTI1_IRQHandler(){

//	Prameter.angle_current=250;
//	EXTI_ClearITPendingBit(EXTI_Line1);
	return;
#if 0
	if(Prameter.staus==1){

	delay_ms(500);
	EXTI_ClearITPendingBit(EXTI_Line1);
	return ;
	}
	if(Tst_Mode==0){


	MOTORRESET;
	MOTORENABLE;
	DIRNEGTIVE;
	Motor_Start();
	Speed_Change(200);
	RED;
	BLUE;

	}
	else{

	if(Tst_Mode!=2){

	PAUSE();
	MOTORRESET;
	MOTORENABLE;
	DIRNEGTIVE;
	Motor_Start();
	Speed_Change(200);
	RED;
	BLUE;
	}

	}
	Tst_Mode=2;
	delay_ms(500);
	EXTI_ClearITPendingBit(EXTI_Line1);

#endif	
}

void EXTI2_IRQHandler(){
	
//	Prameter.angle_current=0;
//	EXTI_ClearITPendingBit(EXTI_Line2);
//	return;
	if(Prameter.staus==1){
	
			delay_ms(500);
			EXTI_ClearITPendingBit(EXTI_Line2);
		  return ;
	}

	Tst_Mode=0;
	PAUSE();
	delay_ms(500);
	EXTI_ClearITPendingBit(EXTI_Line2);
}

