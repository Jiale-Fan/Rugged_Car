/*******************************************************************************
* 实验名称  	: PWM呼吸灯实验
* 实验说明  : 
* 接线说明  : 1，LED模块-->单片机底座管脚（STM32核心板）
					D2-->P16（PA6）
						
* 实验现象	: 开发板上D2指示灯由暗变亮，再由亮变暗，呈现呼吸灯效果
*******************************************************************************/

#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "pwm_motor.h"
#include "encoder.h"
#include "pid.h"
//#include "usart.h"
#include "communicate.h"
#include <stdio.h>

int p=0;

int main()
{
	char sendCount=0;
	
	SysTick_Init(72);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
	
	Motor_Init(3600, 1-1); //20Khz
	Servo_Init(2000, 720-1); //50hz
	Encoder_Init_TIM4();
	PID_Init();
	Communicate_init(9600);
	//USART1_Init(9600);
	
	Set_Speed(0);
	Set_Angle(90);
	
	while(1)
	{
		//Set_Angle(90);
		
		//delay_ms(1000);
		//printf("speedset=%d#", speedSet);
		//p=0;
		
		
		//打印速度
		/*
		delay_ms(100);
		//printf("speednow=%d#", speedNow);
		delay_ms(100);
//		printf("p=%d\r\n", p);
		p ++;
		if (p > 50)
		{
			if (speedSet == 3500)
			{
				speedSet = 1000;
			}				
			else
			{
				speedSet = 3500;
			}
			p = 0;
		}
		*/
		
		/*
		//给树莓派发送速度，角度,这里速度已经乘以1000
		if(sendCount==0)//发送  14.4ms  发送一次数据 70Hz 左右
		{
			//发送需要一定的延时
			//usartSendData(USART1,(short)speedNow,(short)servoNow,sendCtrlFlag);  //1ms
			//printf("pwmnow=%d",pwmNow);
			sendCount++;
		}
		else
		{
			sendCount++;
			if(sendCount==25)
				sendCount=0;
		}
		//获取角度		
		servoNow = servoSet;
		*/
		
	}
}

void TIM3_IRQHandler(void)                            //TIM3中断
{
	static int i=0;
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否
	{
		i++;
		if (i==100)
		{
			//p++;
			i=0;
			Get_Motor_Speed(&speedNow);
			pid_Task.speedSet  = speedSet;
			pid_Task.speedNow  = speedNow;
			Pid_Ctrl(&pwmNow);
			//printf("pwmnow=%d#",pwmNow);
			usartSendData(USART1,(short)speedNow,(short)servoNow,sendCtrlFlag);
			Set_Speed(pwmNow);
			servoNow = servoSet;
			Set_Angle(servoNow);
			//delay_ms(1);
			//printf("pwmnow=%d",pwmNow);
			
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);   //清除TIMx的中断待处理位
			
	}
}
 

///*
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	 {
		 //p++;
		 usartReceiveOneData(USART1,&speedSet,&servoSet,&receCtrlFlag);
		 //usartSendData(USART1,(short)speedNow,(short)servoNow,sendCtrlFlag);
		 USART_ClearITPendingBit(USART1,USART_IT_RXNE);//首先清除中断标志位
	 }
}
//*/
