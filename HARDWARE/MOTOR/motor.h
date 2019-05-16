#ifndef _MOTOR_H
#define _MOTOR_H
#include "stm32f2xx.h"
#include "delay.h"


#define am 								10000.0                 //�Ӽ��ٶ�
#define am2 							100000000.0            //�Ӽ��ٶȵ�ƽ��
#define puper  							100                 //ÿȦ�����������������
#define DE_RATE 						180.0f             //����ļ��ٱ�
#define PUS_PER 						400.0f             //�����תһȦ��Ҫ������
#define MOTOR_START_SPEED 				50.0    //�Ӽ��ٵĿ�ʼ�ٶ�
#define MOTOR_MAX_SPEED   				3000.0   //�Ӽ�������ļ����ٶ�
#define MOTORENABLE  					GPIO_SetBits(GPIOC, GPIO_Pin_9)        //���ʹ��
#define MOTORDISABLE 					GPIO_ResetBits(GPIOC, GPIO_Pin_9)      //���ʧ��
#define DIRPOSITIVE 		 			GPIO_SetBits(GPIOC, GPIO_Pin_8)        //���������
#define DIRNEGTIVE   					GPIO_ResetBits(GPIOC, GPIO_Pin_8)      //������򣺸�
#define MOTORRESET  					{GPIO_ResetBits(GPIOB, GPIO_Pin_14);delay_ms(200);GPIO_SetBits(GPIOB, GPIO_Pin_14);}   //�����λ

typedef struct{
	float angle_last; //��һ�εĽǶ�
	float angle_current;//��ǰҪ��Ƕ�
	float angle;//������ʵʱ�Ƕ�
	u8 flag;
	int speed;  
	u32 ID;     //CAN���ߵ�
	u8 staus;   //�����ǰ��״̬ 0��û������ 1��������
}Motor_Pra;

extern void TIM1_PWM_Init(u32 arr,u32 psc);
extern void TIM3_Int_Init(u16 arr,u16 psc);
extern void Acde_Test(u32 plus,u16 vmax);
extern void MotorGpioInit(void);
extern void CanSend(void);
extern void Motor_Start(void);
extern void Speed_Change(u16 time);
extern void PAUSE(void);
extern void ID_GPIOInit(void);
extern u32  Get_ID(void);
extern void  start_acc(float vmax,float vmin,float ang);

#endif
