#ifndef _MOTOR_H
#define _MOTOR_H
#include "stm32f2xx.h"
#include "delay.h"


#define am 								10000.0                 //加加速度
#define am2 							100000000.0            //加加速度的平方
#define puper  							100                 //每圈编码器输出的脉冲数
#define DE_RATE 						180.0f             //电机的减速比
#define PUS_PER 						400.0f             //电机旋转一圈需要的脉冲
#define MOTOR_START_SPEED 				50.0    //加减速的开始速度
#define MOTOR_MAX_SPEED   				3000.0   //加减速允许的极限速度
#define MOTORENABLE  					GPIO_SetBits(GPIOC, GPIO_Pin_9)        //电机使能
#define MOTORDISABLE 					GPIO_ResetBits(GPIOC, GPIO_Pin_9)      //电机失能
#define DIRPOSITIVE 		 			GPIO_SetBits(GPIOC, GPIO_Pin_8)        //电机方向：正
#define DIRNEGTIVE   					GPIO_ResetBits(GPIOC, GPIO_Pin_8)      //电机方向：负
#define MOTORRESET  					{GPIO_ResetBits(GPIOB, GPIO_Pin_14);delay_ms(200);GPIO_SetBits(GPIOB, GPIO_Pin_14);}   //电机复位

typedef struct{
	float angle_last; //上一次的角度
	float angle_current;//当前要求角度
	float angle;//编码器实时角度
	u8 flag;
	int speed;  
	u32 ID;     //CAN总线的
	u8 staus;   //电机当前的状态 0：没有任务 1：有任务
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
