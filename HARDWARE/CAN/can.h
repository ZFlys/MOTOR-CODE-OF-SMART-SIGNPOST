#ifndef _CAN_H
#define _CAN_H
#include "motor.h"
typedef enum {
	CAN_CFG_RRT_500K = 0,
	CAN_CFG_RRT_1M, 
} BDRT_CFG;
							
extern volatile Motor_Pra Prameter;
u8 CAN_Mode_Init(BDRT_CFG brt_cfg);
extern void Can_Send(char *p);
extern void SennZeroFinish(void);
extern void get_pra(void);
void Can_Send(char *p);

#endif
