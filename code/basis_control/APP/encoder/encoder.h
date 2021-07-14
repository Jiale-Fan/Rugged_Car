#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h" 

#define ENCODER_TIM_PERIOD (u16)(4096*4)   //不可大于65535 因为F103的定时器是16位的。

void Encoder_Init_TIM4(void);
s16 getTIMx_DetaCnt(TIM_TypeDef * TIMx);
void Get_Motor_Speed(int *motorSpeed);

#endif
