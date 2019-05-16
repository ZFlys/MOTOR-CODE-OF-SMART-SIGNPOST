#include "motor.h"
#include "stm32f2xx.h"
#include "can.h"
#include "led.h"
#include "msg.h"
#include <stdio.h>
#include <string.h>
volatile Motor_Pra Prameter;   //����Ĳ���
CanTxMsg SendMsg;     //CAN���ͽṹ��
CanRxMsg GetMsg;      //CAN���սṹ��

/*****************************************/
/*****************************************/
/*****CAN���߳�ʼ��***********************/
/*****************************************/
u8 CAN_Mode_Init(BDRT_CFG brt_cfg)
{
	u8 tsjw, tbs2, tbs1, mode;
	u16 brp;
	u16 mask;
	GPIO_InitTypeDef 		GPIO_InitStructure; 
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;

	NVIC_InitTypeDef  		NVIC_InitStructure;


	if(brt_cfg ==  CAN_CFG_RRT_500K) {
		tsjw = CAN_SJW_1tq;
		tbs2 = CAN_BS2_6tq;
		tbs1 = CAN_BS1_8tq;
		mode = CAN_Mode_Normal;
		brp = 4;
	} else if(brt_cfg ==  CAN_CFG_RRT_1M) {
		tsjw = CAN_SJW_1tq;
		tbs2 = CAN_BS2_6tq;
		tbs1 = CAN_BS1_8tq;
		mode = CAN_Mode_Normal;
		brp = 2;
	} else {
		printf("Can module initialize failed! Has no this brdrate!\r\n");
		return 0xff;
	}
	

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2 | RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//�����������
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);			//��ʼ��IO
	
 

	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;			//��ʱ�䴥��ͨ��ģʽ  
	CAN_InitStructure.CAN_ABOM=DISABLE;			//����Զ����߹���	 
	CAN_InitStructure.CAN_AWUM=DISABLE;			//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=ENABLE;			//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;		 	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=DISABLE;			//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= mode;	        //ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; 
	//���ò�����
 
	CAN_InitStructure.CAN_SJW=tsjw;				//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;        //��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN2, &CAN_InitStructure);        	//��ʼ��CAN1 
  
	CAN_InitStructure.CAN_SJW=tsjw;				//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;        //��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN1, &CAN_InitStructure);        	//��ʼ��CAN1 


	CAN_FilterInitStructure.CAN_FilterNumber=14;	//������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//����λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 	//32λ�� 
	CAN_FilterInitStructure.CAN_FilterIdHigh=((Prameter.ID<<3) >>16) &0xffff;  //????????????  
	CAN_FilterInitStructure.CAN_FilterIdLow=(u16)(Prameter.ID<<3) | CAN_ID_EXT;  
	mask =(Prameter.ID<<18);
	mask ^=Prameter.ID;  
	mask =~mask;  
	mask <<=3;
	mask |=0x02;  
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=(mask>>16)&0xffff; //??????????  
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=mask&0xffff;   //??????????  

	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0

	CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��
	

	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);				//FIFO0��Ϣ�Һ��ж�����.		    

	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // �����ȼ�Ϊ2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;            // �����ȼ�Ϊ2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	return 0;
}   

/*************************************************/
/*************************************************/
/*********CAN���͵ײ㺯��*************************/
/*************************************************/
void Can_Send(char *p){
	
			u16 i=0;
			u32 j=0;
			uint8_t kk;
			SendMsg.StdId=Prameter.ID;
			SendMsg.ExtId=Prameter.ID;
			SendMsg.RTR=CAN_RTR_Data;
			SendMsg.IDE=CAN_Id_Standard;
			SendMsg.DLC=8;
			for(i=0;i<8;i++){ 
				SendMsg.Data[i]=p[i];
			}
			kk=CAN_Transmit(CAN2,&SendMsg);
			while((CAN_TransmitStatus(CAN2,kk)==CAN_TxStatus_Failed)&&(j<0XFFF))
				j++;
			if(j>=0xffff)
				printf("CAN SEND ERROR��\n");
			else
			    printf("CAN�������ݳɹ���\n");
}
/****************************************************/
/****************************************************/
/**********CAN���ݴ�����***************************/
/**********��ȡ����**********************************/
void Data_Proc()
{
	u16 temp;
	u16 speed;
	float angle;
//	char i;

	if((GetMsg.StdId == Prameter.ID)&&(GetMsg.Data[0]==0x5a)&&(GetMsg.Data[7] == 0xa5)) {
		speed = GetMsg.Data[1]<<8|GetMsg.Data[2];
		temp = GetMsg.Data[5]<<8|GetMsg.Data[6];
		angle = (float)(GetMsg.Data[3]<<8|GetMsg.Data[4])+temp/1000.0;//3λС�� 
		Prameter.angle_last=Prameter.angle_current;
		Prameter.speed=speed;
		Prameter.angle_current=angle;
		
		//GetMsg.StdId = 0;
		Prameter.flag = 1;
//		memset((char *)&GetMsg, 0, sizeof(GetMsg));
	}
}



/**************************************/
/**************************************/
/******��Ϣ�����л�ȡ����**************/
/******��ǰδ��************************/
void get_pra(){
	
		 Prameter.angle_last=Prameter.angle_current;
		// GM_Queue_Dequeue(&(Prameter.speed),&(Prameter.angle_current));
}

/**************************************/
/**************************************/
/******CAN���߽����ж�*****************/
/**************************************/
void CAN2_RX0_IRQHandler(void)
{
	//printf("CAN���յ�����\n");
	RED;
	
//	GetMsg.StdId = 0;
	CAN_Receive(CAN2, CAN_FIFO0, &GetMsg);
	
	Data_Proc();
}
