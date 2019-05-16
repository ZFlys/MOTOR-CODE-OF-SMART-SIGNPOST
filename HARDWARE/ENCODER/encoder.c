#include "stm32f2xx.h"
#include "encoder.h"
#include "motor.h"
#include "can.h"
#include "key.h"
volatile  u16 direction;  //电机的旋转方向，通过正交编码器读入
volatile u32 quan=20000; //电机当前的圈数，用于计算位置
volatile u32 count=0;    //旋转的脉冲数，通过正交编码器反馈
volatile u8 ToZero=0;    //零位标志
u8 ll=100;
/****************************************************/
/****************************************************/
/*********过零信号IO初始化***************************/
/****************************************************/
void Gpio_ZeroInit(){
	
			GPIO_InitTypeDef  GPIO_InitStructure;
			NVIC_InitTypeDef NVIC_InitStructure;
			EXTI_InitTypeDef exti;
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //使能PB,PC端口时钟
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				 //LEDR-->PB.0 端口配置
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 		 //推挽输出
			//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
			//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOC, &GPIO_InitStructure);		
			SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource10);
			exti.EXTI_Line=EXTI_Line10;
			exti.EXTI_Mode=EXTI_Mode_Interrupt;
			exti.EXTI_Trigger=EXTI_Trigger_Rising;
			exti.EXTI_LineCmd=ENABLE;
			EXTI_Init(&exti);
			NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;  
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
			NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;  //从优先级3级
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
			NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
	
}
/*********************************************************/
/*********************************************************/
/**************过零中断服务函数***************************/
/*********************************************************/
void EXTI15_10_IRQHandler(){

   if(ToZero==0){
	    PAUSE();
	    ToZero=1;
		  quan=20000;
		  TIM8->CNT=0;
	  
	 }
   else{
	 
	  if(direction==FORWARD){
	   quan=20000;
		  TIM8->CNT=0;
		 count=0;
		 Prameter.angle=0;
		}
		else{
		  quan=20800;
		  TIM8->CNT=99;
		  count=0;
		  Prameter.angle=360;
		 }
	 
	 }
   EXTI_ClearITPendingBit(EXTI_Line10);
}
/*****************************************************/
/*****************************************************/
/*******编码器初始化**********************************/
/*****************************************************/
void TIM8_Mode_Config(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;
        TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
        NVIC_InitTypeDef NVIC_InitStructure;
        TIM_ICInitTypeDef        TIM8_ICInitStructure;
        
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);          //TIM5????    
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);         //??PORTA??        
        
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //GPIOC6/7
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//????
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;        //??100MHz
        //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //??????
        //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //??
        GPIO_Init(GPIOC,&GPIO_InitStructure); //???PAC

        GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8); //PC6??????8
        GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8); //PC7??????8

          
        TIM_TimeBaseStructure.TIM_Prescaler=0;  //?????
        TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //??????
        TIM_TimeBaseStructure.TIM_Period=(puper)-1;   //??????
        TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
        
        TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);

        TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
        TIM_ICStructInit(&TIM8_ICInitStructure);
        TIM8_ICInitStructure.TIM_Channel = TIM_Channel_1;
        TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //???TI1?
        TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;         //??????,??? 
        TIM8_ICInitStructure.TIM_ICFilter = 0x6;
        TIM_ICInit(TIM8, &TIM8_ICInitStructure);
        TIM8_ICInitStructure.TIM_Channel = TIM_Channel_2;
        TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //???TI1?
        TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;         //??????,??? 
        TIM8_ICInitStructure.TIM_ICFilter = 0x6;
        TIM_ICInit(TIM8, &TIM8_ICInitStructure);        
        
        
        TIM_SetCounter(TIM8,0);        
  //TIM_ITConfig(TIM8,TIM_IT_Update|TIM_IT_CC1,ENABLE);//?????? ,??CC1IE????        
        TIM_ITConfig(TIM8,TIM_IT_Update,ENABLE);//?????? ,??CC1IE????                
        TIM_Cmd(TIM8,ENABLE );         //?????5
        NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
        direction = (TIM8->CR1 & TIM_CR1_DIR ? FORWARD : BACKWARD);
      NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//?????3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;                //????3
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                        //IRQ????
        NVIC_Init(&NVIC_InitStructure);        //??????????VIC????
        
        
}


/**************************************************************/
/**************************************************************/
/****************编码器中断服务程序****************************/
/**************************************************************/
void TIM8_UP_TIM13_IRQHandler(void)
{                     

         
                
  if(TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
        {        
                              
                 
                direction = (TIM8->CR1 & TIM_CR1_DIR ? FORWARD : BACKWARD);
					      if(direction==FORWARD)
									 quan++;
								else
									 quan--;
									 
         }                                                                    
               
   if(quan>20000){
					    if(direction==FORWARD)
						    count = TIM8->CNT+(quan-20000)*puper;
					    else
							  count =puper-TIM8->CNT+(quan-20000)*puper;
							count/=8;
					    Prameter.angle=count/(DE_RATE*puper)*360.0;  
					  
					    
					}
					else{
					
					    if(direction==FORWARD)
						     count = (20000-quan)*puper+TIM8->CNT;
					    else
							  count =(20000-quan)*puper+puper-TIM8->CNT;
							count/=8;
					    Prameter.angle=360-count/(DE_RATE*puper)*360.0;	}
			
     TIM_ClearITPendingBit(TIM8, TIM_IT_Update); //???????
}
void TIM8_CC_IRQHandler(void)
{                     

         
                if(TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET)//??
                {             
               //  printf("TIM8_CC1_IRQHandler=%d\r\n",TIM8->CNT );//                
                 TIM_ClearITPendingBit(TIM8, TIM_IT_CC1); //???????
                }
                if(TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)//??1??????
                {        
              //  printf("TIM8_CC2_IRQHandler=%d\r\n",TIM8->CNT );//                
                TIM_ClearITPendingBit(TIM8, TIM_IT_CC2); //???????
                }else                                                                  
                        {
                                
                        //        TIM_Cmd(TIM8,DISABLE );         //?????5
                         //        TIM_SetCounter(TIM8,0);
                                 
                        //        TIM_Cmd(TIM8,ENABLE );         //?????5
                        }                    
                                                                            
         
        //TIM_ClearITPendingBit(TIM8, TIM_IT_CC1|TIM_IT_Update); //???????
        
}

