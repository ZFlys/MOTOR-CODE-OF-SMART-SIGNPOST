#ifndef _ENCODER_H
#define _ENCODER_H
#include "stm32f2xx.h"
#define IsZero GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)

 enum{


        FORWARD=1,
        BACKWARD=0,

};
extern volatile u32 quan;
extern volatile u32 count;
extern volatile u16 direction;
extern void Gpio_ZeroInit(void);
extern volatile u8 ToZero;
extern void TIM8_Mode_Config(void);
#endif
