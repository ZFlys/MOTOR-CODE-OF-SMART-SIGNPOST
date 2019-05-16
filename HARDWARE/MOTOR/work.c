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

void task(void);   //��������
float angle_error=0;
 

/*********************************/
/*********************************/
/*****ϵͳ��ʼ��������************/
void system_init(){
   
	/*******��ʱ������ʼ��*********/
	delay_init();	
	
	/*******�жϷ�������***********/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	
	/*******LEDָʾ�Ƴ�ʼ��********/
	LED_Init();   
	
	/*******���ڵ��Գ�ʼ��*********/
	/*******����ʱû����***********/
	uart_init(9600);
	
	/*******�����źų�ʼ�����жϷ�ʽ��************/
	Gpio_ZeroInit();
	
	/*******CAN����ID��ʼ�������뿪�أ�***********/
	ID_GPIOInit();
	
	/*******CAN���߳�ʼ��***********/
	CAN_Mode_Init(CAN_CFG_RRT_500K);
	
	/*******��ȡCAN����ID***********/
	Prameter.ID = Get_ID();
	
  /*******PWM��ʼ����������ƣ�***/
 	TIM1_PWM_Init(0,59);
	
	/*******��ʱ����ʼ����ʱ�������**************/
    TIM3_Int_Init(59,99);
  
	/*******��������߳�ʼ��********/  
	MotorGpioInit();
	
	
	if(ToZero==0){  //�ж��Ƿ���Ҫ����
	
		/*******************************/
		/****���õ��ʹ�ܺͷ���*********/
		MOTORRESET;
		MOTORENABLE;
		DIRPOSITIVE;
		/*******************************/

		/*******************************/	
		/*****���������ʼ����**********/		
		Motor_Start();
		Speed_Change(500);
		BLUE;
		/*******************************/
	}

	while(ToZero==0);  	//�ȴ��������
	/*****���ͻ�������ź�**********/	
//	SennZeroFinish();
	GREEN;
	
	/********��������ʼ����λ�ò�����*******/
	TIM8_Mode_Config();
	
	/********��ʼ������*********************/
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

  /******˵�������Գ���**********************/
/*���ض�λ180�Ⱥ�30��***********************/	
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
/*********����������*********************/
void work()
{
	system_init();
	GREEN;

	while(1) {

		if(Prameter.flag == 1){ 		//������
			RED;
			
			//get_pra();
			if(Prameter.angle_current<Prameter.angle) {//Ĭ����ת�ĽǶ�

				/*****��ת*****/
				if((Prameter.angle-Prameter.angle_current) > 180) {
					test_f = 1;
					angle_error = 360 + Prameter.angle_current - Prameter.angle;
					if(angle_error>2){
						MOTORENABLE;
						DIRPOSITIVE;
						MOTORRESET;
						printf("��ת:%3.2f��\n",angle_error);

						start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
						PAUSE();
						CanSend();
						BLUE;
						printf("��ǰ�Ƕ�:%3.2f��",Prameter.angle_current);
					}

				} else {
					test_f=2; 
					angle_error = Prameter.angle - Prameter.angle_current;
					if(angle_error>2){
						MOTORENABLE;
						DIRNEGTIVE;
						MOTORRESET;
						printf("��ת:%3.2f��\n",angle_error);

						start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
						PAUSE();
						CanSend();
						BLUE;
						printf("��ǰ�Ƕ�:%3.2f��",Prameter.angle_current);
					}
				}

			} else {

				if((Prameter.angle_current-Prameter.angle)>180){

					/*****��ת*****/
					test_f=3;
					angle_error = 360 - Prameter.angle_current + Prameter.angle;
					if(angle_error>2){
						MOTORENABLE;
						DIRNEGTIVE;
						MOTORRESET;
						printf("��ת:%3.2f��\n",angle_error);
						start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
						PAUSE();
						CanSend();
						BLUE;
						printf("��ǰ�Ƕ�:%3.2f��",Prameter.angle_current);
					}
				} else {

					/*****��ת*****/
					test_f=4;
					angle_error=Prameter.angle_current-Prameter.angle;
					if(angle_error>2){
						MOTORENABLE;
						DIRPOSITIVE;
						MOTORRESET;
						printf("��ת:%3.2f��\n",angle_error);
						start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
						PAUSE();
						CanSend();
						BLUE;
						printf("��ǰ�Ƕ�:%3.2f��",Prameter.angle_current);
					}
				}					
			}
			
			Prameter.flag = 0;    //������
		} // end if //������
	}  // end while(1)
}


/*******************************************/
/*******************************************/
/******���Գ���*****************************/

void task(){
  RED;
			 
		      //get_pra();
		      if(Prameter.angle_current<Prameter.angle){//Ĭ����ת�ĽǶ�
					
					  /*****��ת*****/
						   if((Prameter.angle-Prameter.angle_current)>180){
							 
							  angle_error=360+Prameter.angle_current-Prameter.angle;
								if(angle_error>2){
											MOTORENABLE;
											DIRPOSITIVE;
											MOTORRESET;
											printf("��ת:%3.2f��\n",angle_error);
											//Acde_Test(angle_error/360.0*DE_RATE*PUS_PER,Prameter.speed);
											start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
									    PAUSE();
//											CanSend();
											BLUE;
											printf("��ǰ�Ƕ�:%3.2f��",Prameter.angle_current);}
							 
							 }
							 else{
							
								angle_error=Prameter.angle-Prameter.angle_current;
								if(angle_error>2){
											MOTORENABLE;
											DIRNEGTIVE;
											MOTORRESET;
											printf("��ת:%3.2f��\n",angle_error);
											//Acde_Test(angle_error/360.0*DE_RATE*PUS_PER,Prameter.speed);
											start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
								    	PAUSE();
//											CanSend();
											BLUE;
											printf("��ǰ�Ƕ�:%3.2f��",Prameter.angle_current);}
							 }
						    
					}
					else{
					
					     if((Prameter.angle_current-Prameter.angle)>180){
							 
							 /*****��ת*****/
							
								     angle_error=360-Prameter.angle_current+Prameter.angle;
								     if(angle_error>2){
														 MOTORENABLE;
														 DIRNEGTIVE;
														 MOTORRESET;
														 printf("��ת:%3.2f��\n",angle_error);
														start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
														PAUSE();
//														CanSend();
														BLUE;
														printf("��ǰ�Ƕ�:%3.2f��",Prameter.angle_current);
										 }
							 }
               else{
							 
							 /*****��ת*****/
								  
								     angle_error=Prameter.angle_current-Prameter.angle;
								     if(angle_error>2){
													 MOTORENABLE;
													 DIRPOSITIVE;
													 MOTORRESET;
													 printf("��ת:%3.2f��\n",angle_error);
											 	   start_acc(MOTOR_MAX_SPEED,MOTOR_START_SPEED,angle_error);
											       PAUSE();
//											  	   CanSend();
													 BLUE;
													 printf("��ǰ�Ƕ�:%3.2f��",Prameter.angle_current);
										 }
							 }					
					}

}
