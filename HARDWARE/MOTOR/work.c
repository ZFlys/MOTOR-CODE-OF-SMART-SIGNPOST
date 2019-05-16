#include "motor.h"
#include "msg.h"
#include "led.h"
#include "can.h"
#include "motor.h"
#include "encoder.h"
#include "usart.h"
#include "work.h"
#include "motor.h"
#include "stm32f2xx.h"
#include "key.h"

void task(void);   //测试任务
float angle_error=0;
 

/*********************************/
/*********************************/
/*****系统初始化化函数************/
void system_init(){
   
	/*******延时函数初始化*********/
	delay_init();	
	
	/*******中断分组配置***********/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	
	/*******LED指示灯初始化********/
	LED_Init();   
	
	/*******串口调试初始化*********/
	/*******工作时没有用***********/
	uart_init(9600);
	
	/*******回零信号初始化（中断方式）************/
	Gpio_ZeroInit();
	
	/*******CAN总线ID初始化（拨码开关）***********/
	ID_GPIOInit();
	
	/*******CAN总线初始化***********/
	CAN_Mode_Init(CAN_CFG_RRT_500K);
	
	/*******获取CAN总线ID***********/
	Prameter.ID = Get_ID();
	
  /*******PWM初始化（电机控制）***/
 	TIM1_PWM_Init(0,59);
	
	/*******定时器初始化（时间计数）**************/
    TIM3_Int_Init(59,99);
  
	/*******电机控制线初始化********/  
	MotorGpioInit();
	
	
	if(ToZero==0){  //判断是否需要归零
	
		/*******************************/
		/****设置电机使能和方向*********/
		MOTORRESET;
		MOTORENABLE;
		DIRPOSITIVE;
		/*******************************/

		/*******************************/	
		/*****启动电机开始回零**********/		
		Motor_Start();
		Speed_Change(500);
		BLUE;
		/*******************************/
	}

	while(ToZero==0);  	//等待回零完成
	/*****发送回零完成信号**********/	
//	SennZeroFinish();
	GREEN;
	
	/********编码器初始化（位置测量）*******/
	TIM8_Mode_Config();
	
	/********初始化参数*********************/
	quan=20000;
	count=0;
	MOTORDISABLE;
	Prameter.angle_last=0;
	Prameter.angle_current=0;
	Prameter.flag=0;
	Prameter.angle=0;
	Prameter.staus=0;
	
	/***************************************/
	Key_GpioConfig();
	direction = 2;

  /******说明：测试程序**********************/
/*来回定位180度和30度***********************/	
/*	
	while(1){
		Prameter.angle_current=180;
	  task();
	
	  delay_ms(1000);
		delay_ms(1000);
    Prameter.angle_current=150;
	  task();
	
	  delay_ms(1000);
		delay_ms(1000);
		 Prameter.angle_current=270;
	  task();
	
	  delay_ms(1000);
		delay_ms(1000);
		
		 Prameter.angle_current=60;
	  task();
	
	  delay_ms(1000);
		delay_ms(1000);
		 Prameter.angle_current=130;
	  task();
	
	 delay_ms(1000);
		delay_ms(1000);delay_ms(1000);
			
	}
*/
	

}

u8 test_f=0;

/******************************************/
/*********任务工作函数*********************/
void work()
{
	system_init();
	GREEN;

	while(1) {

		if(Prameter.flag == 1){ 		//有任务
			RED;
			
			//get_pra();
			if(Prameter.angle_current<Prameter.angle) {//默认正转的角度

				/*****反转*****/
				if((Prameter.angle-Prameter.angle_current) > 180) {
					test_f = 1;
					angle_error = 360 + Prameter.angle_current - Prameter.angle;
					if(angle_error>2){
						MOTORENABLE;
						DIRPOSITIVE;
						MOTORRESET;
						printf("反转:%3.2f度\n",angle_error);

						start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
						PAUSE();
						CanSend();
						BLUE;
						printf("当前角度:%3.2f度",Prameter.angle_current);
					}

				} else {
					test_f=2; 
					angle_error = Prameter.angle - Prameter.angle_current;
					if(angle_error>2){
						MOTORENABLE;
						DIRNEGTIVE;
						MOTORRESET;
						printf("反转:%3.2f度\n",angle_error);

						start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
						PAUSE();
						CanSend();
						BLUE;
						printf("当前角度:%3.2f度",Prameter.angle_current);
					}
				}

			} else {

				if((Prameter.angle_current-Prameter.angle)>180){

					/*****反转*****/
					test_f=3;
					angle_error = 360 - Prameter.angle_current + Prameter.angle;
					if(angle_error>2){
						MOTORENABLE;
						DIRNEGTIVE;
						MOTORRESET;
						printf("反转:%3.2f度\n",angle_error);
						start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
						PAUSE();
						CanSend();
						BLUE;
						printf("当前角度:%3.2f度",Prameter.angle_current);
					}
				} else {

					/*****正转*****/
					test_f=4;
					angle_error=Prameter.angle_current-Prameter.angle;
					if(angle_error>2){
						MOTORENABLE;
						DIRPOSITIVE;
						MOTORRESET;
						printf("正转:%3.2f度\n",angle_error);
						start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
						PAUSE();
						CanSend();
						BLUE;
						printf("当前角度:%3.2f度",Prameter.angle_current);
					}
				}					
			}
			
			Prameter.flag = 0;    //清任务
		} // end if //有任务
	}  // end while(1)
}


/*******************************************/
/*******************************************/
/******测试程序*****************************/

void task(){
  RED;
			 
		      //get_pra();
		      if(Prameter.angle_current<Prameter.angle){//默认正转的角度
					
					  /*****反转*****/
						   if((Prameter.angle-Prameter.angle_current)>180){
							 
							  angle_error=360+Prameter.angle_current-Prameter.angle;
								if(angle_error>2){
											MOTORENABLE;
											DIRPOSITIVE;
											MOTORRESET;
											printf("反转:%3.2f度\n",angle_error);
											//Acde_Test(angle_error/360.0*DE_RATE*PUS_PER,Prameter.speed);
											start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
									    PAUSE();
//											CanSend();
											BLUE;
											printf("当前角度:%3.2f度",Prameter.angle_current);}
							 
							 }
							 else{
							
								angle_error=Prameter.angle-Prameter.angle_current;
								if(angle_error>2){
											MOTORENABLE;
											DIRNEGTIVE;
											MOTORRESET;
											printf("反转:%3.2f度\n",angle_error);
											//Acde_Test(angle_error/360.0*DE_RATE*PUS_PER,Prameter.speed);
											start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
								    	PAUSE();
//											CanSend();
											BLUE;
											printf("当前角度:%3.2f度",Prameter.angle_current);}
							 }
						    
					}
					else{
					
					     if((Prameter.angle_current-Prameter.angle)>180){
							 
							 /*****反转*****/
							
								     angle_error=360-Prameter.angle_current+Prameter.angle;
								     if(angle_error>2){
														 MOTORENABLE;
														 DIRNEGTIVE;
														 MOTORRESET;
														 printf("反转:%3.2f度\n",angle_error);
														start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
														PAUSE();
//														CanSend();
														BLUE;
														printf("当前角度:%3.2f度",Prameter.angle_current);
										 }
							 }
               else{
							 
							 /*****正转*****/
								  
								     angle_error=Prameter.angle_current-Prameter.angle;
								     if(angle_error>2){
													 MOTORENABLE;
													 DIRPOSITIVE;
													 MOTORRESET;
													 printf("正转:%3.2f度\n",angle_error);
											 	   start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
											       PAUSE();
//											  	   CanSend();
													 BLUE;
													 printf("当前角度:%3.2f度",Prameter.angle_current);
										 }
							 }					
					}

}
