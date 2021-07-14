#ifndef _pwm_H
#define _pwm_H

#include "system.h"

#define DIRECT_PORT 			GPIOA   
#define DIRECT_PIN 			GPIO_Pin_0
#define DIRECT_PORT_RCC		RCC_APB2Periph_GPIOA
#define DIRECT PAout(0)

extern int pwmNow;
extern int speedNow;
extern int speedSet;
extern int servoNow;
extern int servoSet;

void Motor_Init(u16 per,u16 psc);
void Servo_Init(u16 per,u16 psc);
void Set_Speed(int s);
void Set_Angle(u16 a);

#endif
