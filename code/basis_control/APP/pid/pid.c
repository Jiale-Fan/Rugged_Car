#include "pid.h"
//#include "usart.h"
#include <stdio.h>

struct pid_uint pid_Task;

/****************************************************************************
*函数名称：PID_Init(void)
*函数功能：初始化PID结构体参数
****************************************************************************/

void PID_Init(void)
{
//乘以1024原因避免出现浮点数运算，全部是整数运算，这样PID控制器运算速度会更快
/***********************左轮速度pid****************************/
	pid_Task.Kp = 1024 * 0.08;//0.4
 	pid_Task.Ki = 1024 * 0;	
	pid_Task.Kd = 1024 * 0.01; 
	pid_Task.Ur = 1024 * 359;
	pid_Task.Adjust   = 0;
	pid_Task.En       = 1;
	pid_Task.speedSet = 0;
	pid_Task.speedNow = 0;
	reset_Uk(&pid_Task);		
}

/***********************************************************************************************
 函 数 名：void reset_Uk(PID_Uint *p)
 功    能：初始化U_kk,ekk,ekkk
 说    明：在初始化时调用，改变PID参数时有可能需要调用
 入口参数：PID单元的参数结构体 地址
************************************************************************************************/

void reset_Uk(struct pid_uint *p)
{
	p->U_kk=0;
	p->ekk=0;
	p->ekkk=0;
}

/***********************************************************************************************
 函 数 名：s32 PID_commen(int set,int jiance,PID_Uint *p)
 功    能：数字PID
 说    明：求任意单个PID的控制量
 入口参数：期望值，实测值，PID单元结构体
 返 回 值：PID控制量
************************************************************************************************/

s32 PID_common(int set,int detect,struct pid_uint *p)
{
	int ek=0,U_k=0;

	ek=detect - set;                                                               
	
	U_k=p->U_kk + p->Kp*(ek - p->ekk) + p->Ki*ek + p->Kd*(ek - 2*p->ekk + p->ekkk);
	
	p->U_kk=U_k;
  p->ekkk=p->ekk;
	p->ekk=ek;
	
	if(U_k>(p->Ur))		                                    
		U_k=p->Ur;
	if(U_k<-(p->Ur))
		U_k=-(p->Ur);
	
	//printf("set=%d#now=%d#ek=%d#uk=%d#",set,detect,ek,U_k>>10);
	return U_k>>10; 
}

/***********************************************************************************
** 函数名称 ：void Pid_Which(struct pid_uint *pl, struct pid_uint *pr)
** 函数功能 ：pid选择函数	      
***********************************************************************************/

void Pid_Which(struct pid_uint *p)
{
	/**********************左轮速度pid*************************/
	if(p->En == 1)
	{									
		p->Adjust = -PID_common(p->speedSet, p->speedNow, p);		
	}	
	else
	{
		p->Adjust = 0;
		reset_Uk(p);
		p->En = 2; 
	}
}

/*******************************************************************************
 * 函数名：Pid_Ctrl(int *leftMotor,int  *rightMotor)
 * 描述  ：Pid控制
 *******************************************************************************/

void Pid_Ctrl(int *Motor)
{
	Pid_Which(&pid_Task); 
	*Motor += pid_Task.Adjust;
	if (*Motor < 0)
	{
		*Motor = 0;
	}
	if (*Motor > 3600)
	{
		*Motor = 3599;
	}
}

