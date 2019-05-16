/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/**********电机相关函数*****************************************************/
/**********zfly于2017年11月13日完成***********************************/
/**********版权所有：西南科技大学**********************************/
#include "can.h"
#include "motor.h"
#include "stm32f2xx.h"
#include "encoder.h"
#include "math.h"
#include "key.h"
float vmx=0;
float t=0;
float f=1000;
u32 motor_time=0;
u32 motor_plus=0;
char CANTxData[8];
float vmn;
u32 all_pus;
/****************************/
/****************************/
/******数据上发函数**********/
void CanSend(){
	
	  u16 angle_int,angle_float;
	  u32 tmp=Prameter.angle_current*1000;
	  u16 tmp1=vmx;
	  angle_int=tmp/1000;
	  angle_float=tmp%1000;
		CANTxData[0]=0x5a;
		CANTxData[1]=tmp1/256;
		CANTxData[2]=tmp%256;
		CANTxData[3]=angle_int/256;
		CANTxData[4]=angle_int%256;
		CANTxData[5]=angle_float/256;
		CANTxData[6]=angle_float%256;
		CANTxData[7]=0xa5;	
	  Can_Send(CANTxData);
}
/***********************/
/**********************/
/****回零完成上传函数***/
void SennZeroFinish(void){
     
		CANTxData[0]=0x0F;
		CANTxData[1]=0x00;
		CANTxData[2]=0x00;
		CANTxData[3]=0x00;
		CANTxData[4]=0x00;
		CANTxData[5]=0x00;
		CANTxData[6]=0x00;
		CANTxData[7]=0xF0;	
		Can_Send(CANTxData);
}


/***********************/
/***********************/
/*******电机io初始化********/
void MotorGpioInit(){

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);	 //使能PB,PC端口时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;				 //LEDR-->PB.0 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.0		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
}
/***********************/
/***********************/
/*****PWM初始化*********/
void TIM1_PWM_Init(u32 arr,u32 psc)
{                            
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
     
     
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
     
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
     
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
 
     
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;        
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;    
    TIM_TimeBaseStructure.TIM_Prescaler = psc;                      
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseStructure.TIM_Period = arr;                         
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = (arr+1)/2;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
 
    TIM_OC1Init(TIM1,&TIM_OCInitStructure);
 
    
    TIM_CtrlPWMOutputs(TIM1,DISABLE);
    TIM_Cmd(TIM1,DISABLE);
}

 void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
	
	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


  TIM_Cmd(TIM3,DISABLE);				 
}
/***********************/
/***********************/
/*****速度设置底层函数****/
/*****参数为分频系数******/
void Speed_Change(u16 time)
{
  u16 CCR4_Val=(time+1)/2;

  TIM1->ARR=time;
  TIM1->CCR1=CCR4_Val;
 
 }
/***********************/
/***********************/
/****电机停止函数*******/
void PAUSE(void)
{
	 TIM_ARRPreloadConfig(TIM1,DISABLE);//使能TIM1重装
	 TIM_Cmd(TIM1,DISABLE);
	 TIM_CtrlPWMOutputs(TIM1,DISABLE);
}

/***********************/
/***********************/
/*****计时定时器中断服务程序******/
void TIM3_IRQHandler(void)   //TIM3中断
{

	
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
		{
			 //清除TIMx更新中断标志 
		  motor_time++;
		  t=motor_time*0.0001;
  
		 } 

		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  ); 
}

/***********************/
/***********************/
/******启动电机函数*****/
void Motor_Start(){

		Speed_Change(60000);
		TIM_ARRPreloadConfig(TIM1,ENABLE);//使能TIM1重装
		TIM_Cmd(TIM1,ENABLE);
		TIM_CtrlPWMOutputs(TIM1,ENABLE);
}
/***********************/
/***********************/
/*****电机复位函数******/
void motortest(){

    motor_time=0;
    motor_plus=0;
    t=0;
    Motor_Start();
    TIM_Cmd(TIM3,ENABLE);
	Speed_Change(30000);


}

/********************************************************/
/********************************************************/
/********************************************************/
/*******电机加减速核心函数(采用s曲线加减速)**************/
/*******参数1：开始频率和结束、频率相等******************/
/*******参数2：最大的频率********************************/
/*******参数3：需要旋转的角度(单位：弧度)****************/
void speed_accandc(u32 startf,u32 maxf,u32 pus,float ang){

	u16 div = 0;       //分频系数
	float error = 0;   //加减速的差值
	float l;         //按最大速度要求计算的位移
	float T = 0;       //匀速时间
	float T1 = 0;      //加加速阶段的时间
	float T2,T3,T4,T5;  
	l = (maxf * maxf - startf * startf) * 2.0 / am;
	/******************************************/
	/******************************************/
	/******根据实际情况重新计算最大速度********/
	if(l > pus){
		maxf = sqrt(am * pus / 2.0 + startf * startf);
		T = 0;
	}
	else 
		T = (pus - l) / maxf;

	T1 = (maxf - startf) / (am);
	T2 = 2 * T1;
	T3 = T2 + T;
	T4 = T3 + T1;
	T5 = T4 + T1;	 
	motor_time = 0;
	motor_plus = 0;
	t = 0;
	Motor_Start();
	f=1000;
	TIM_Cmd(TIM3,ENABLE);
	Prameter.staus = 0;

	error = maxf - startf;
	/***************************************/
	/****开始加减速************************/
	while(1){   
		if( Prameter.staus == 1){
			Prameter.staus = 0;
			break;
		} 

		if(t <= T1)
			f = 0.5 * am2 * t * t / error + startf;
		
		else if(t <= T2)
			f = maxf - am2 * 0.5 * (T2 - t) * (T2 - t) / error;
		
		else if(t <= T3)
			f = maxf;

		else if(t <= T4)
			f = maxf - am2 * 0.5 * (t - T3) * (t - T3) / error;
		
		else if(t <= T5)
			f = am2 * (T5 - t) * (T5 - t) / error * 0.5 + startf;
		
		else
			break;

		div = (1000000 / f);
		Speed_Change(div);
	}
}	
/****************************************/
/****************************************/
/********电机加减速开始程序**************/
/****vmax:加速允许的最大速度,单位:r/min***/
/****vmin:加减速的起始速度,单位:r/min****/
/****ang:加减速完成的角度,单位：弧度*****/
  
void start_acc(float vmax,float vmin,float ang){

	  u32 pu;
	  u32 stf,maf;
		stf = vmin / 60.0 * PUS_PER;
		maf = vmax / 60.0 * PUS_PER;
		pu = ang / 360.0 * PUS_PER * DE_RATE;
		speed_accandc(stf ,maf,pu,ang);
	
}
void ID_GPIOInit(){
	
		GPIO_InitTypeDef  GPIO_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //使能PB,PC端口时钟
			
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;				 //LEDR-->PB.0 端口配置
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 		 //推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOC, &GPIO_InitStructure);				 

}

u32 Get_ID()
{
		u32 res;
		u8 in1=0,in2=0,in3=0,in4=0;
		in1=!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0);	
		in2=(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1))<<1;
		in3=(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2))<<2;
		in4=(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3))<<3; 
		res=in1+in2+in3+in4;
		return res;
}
